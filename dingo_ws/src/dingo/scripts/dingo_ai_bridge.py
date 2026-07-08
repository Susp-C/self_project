#!/usr/bin/env python3
"""
AI <-> Joystick arbitration bridge.

- Subscribes:
    /joy_human          sensor_msgs/Joy     (real joystick, remapped)
    /ai_camera/cmd      geometry_msgs/Twist
    /ai_camera/mode     std_msgs/String
    /ai_camera/enable   std_msgs/Bool
- Publishes:
    /joy                sensor_msgs/Joy     (single source for dingo_driver)
    /ai_camera/enable   std_msgs/Bool       (latched, reflects current AI state)

Priority: joystick > AI. If any joystick axis exceeds threshold OR any button
is pressed within the last `joy_active_hold` seconds, joystick passthrough is
used and AI is suppressed. Otherwise AI takes over (trot toggle + cmd).

Mode behaviour:
    FOLLOW : auto-enter TROT, walk toward the tracked person.
    DANCE  : stay in REST, left stick forced to 0, only right stick axes
             (head_yaw / body_wiggle) are used for in-place head/body motion.
    IDLE   : all AI output zeroed, drop out of TROT back to REST.

Key fix: whenever the mode changes, and before toggling TROT, a neutral
(all-zero) Joy frame is published first, so leftover posture/gaze offset from
DANCE does not get carried into the next mode (which previously left the legs
off-center when starting to run).
"""

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Joy axes / buttons (matches InputInterface.py)
AX_LX, AX_LY = 0, 1
AX_RX, AX_RY = 3, 4
AX_DPADX, AX_DPADY = 6, 7
BTN_X, BTN_L1, BTN_R1 = 0, 4, 5
N_AXES, N_BUTTONS = 8, 11


class AIBridge:
    def __init__(self):
        rospy.init_node("dingo_ai_bridge")

        self.enabled = rospy.get_param("~enabled", True)
        self.max_x = rospy.get_param("~max_x", 0.4)
        self.max_y = rospy.get_param("~max_y", 0.3)
        self.max_yaw = rospy.get_param("~max_yaw", 1.0)
        self.cmd_rate = rospy.get_param("~cmd_rate", 30.0)
        self.joy_axis_thresh = rospy.get_param("~joy_axis_thresh", 0.15)
        self.joy_active_hold = rospy.get_param("~joy_active_hold", 1.5)

        self.mode = "IDLE"
        self.cur_cmd = Twist()
        self.last_cmd_t = rospy.Time.now()

        self.last_human_joy = None
        self.last_human_active_t = rospy.Time(0)

        self.trot_active = False
        self.next_trot_toggle = rospy.Time.now()

        self.last_announced_enable = None

        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=4)
        self.enable_pub = rospy.Publisher(
            "/ai_camera/enable", Bool, queue_size=1, latch=True)
        self._publish_enable(self.enabled)

        rospy.Subscriber("/joy_human", Joy, self.on_joy_human)
        rospy.Subscriber("/ai_camera/cmd", Twist, self.on_cmd)
        rospy.Subscriber("/ai_camera/mode", String, self.on_mode)
        rospy.Subscriber("/ai_camera/enable", Bool, self.on_enable)

        rospy.Timer(rospy.Duration(1.0 / self.cmd_rate), self.tick)

        rospy.loginfo(
            "[ai_bridge] up, AI enabled=%s, joy threshold=%.2f, hold=%.1fs",
            self.enabled, self.joy_axis_thresh, self.joy_active_hold,
        )

    # ---------- callbacks ----------
    def on_joy_human(self, msg):
        self.last_human_joy = msg
        if self._joy_is_active(msg):
            self.last_human_active_t = rospy.Time.now()

    def on_cmd(self, msg):
        self.last_cmd_t = rospy.Time.now()
        self.cur_cmd = msg

    def on_mode(self, msg):
        new_mode = msg.data
        if new_mode != self.mode:
            rospy.loginfo("[ai_bridge] mode %s -> %s", self.mode, new_mode)
            # Publish one neutral frame on any mode change so leftover
            # posture/gaze offset from the previous mode (especially DANCE
            # right-stick posture) is cleared before the new mode starts.
            self.joy_pub.publish(self._make_joy())
            self.cur_cmd = Twist()
        self.mode = new_mode

    def on_enable(self, msg):
        # Allow external (e.g. webpage) to flip AI master switch.
        new_val = bool(msg.data)
        if new_val != self.enabled:
            self.enabled = new_val
            rospy.loginfo("[ai_bridge] AI enabled -> %s", self.enabled)

    # ---------- helpers ----------
    def _publish_enable(self, val):
        if self.last_announced_enable == val:
            return
        self.enable_pub.publish(Bool(data=bool(val)))
        self.last_announced_enable = val

    def _joy_is_active(self, msg):
        for a in msg.axes:
            if abs(a) > self.joy_axis_thresh:
                return True
        for b in msg.buttons:
            if b:
                return True
        return False

    def _make_joy(self, axes_dict=None, btn_dict=None):
        j = Joy()
        j.header.stamp = rospy.Time.now()
        j.axes = [0.0] * N_AXES
        j.buttons = [0] * N_BUTTONS
        if axes_dict:
            for i, v in axes_dict.items():
                j.axes[i] = float(v)
        if btn_dict:
            for i, v in btn_dict.items():
                j.buttons[i] = int(v)
        return j

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _trot_toggle_pulse(self):
        """Reset posture first, then send a press/release of R1 to toggle trot."""
        # Neutral frame first: make sure no posture/gaze offset is present
        # when we switch gait, so the legs return to zero position.
        self.joy_pub.publish(self._make_joy())
        rospy.sleep(0.05)
        # Toggle trot gait with R1.
        self.joy_pub.publish(self._make_joy(btn_dict={BTN_R1: 1}))
        rospy.sleep(0.05)
        self.joy_pub.publish(self._make_joy(btn_dict={BTN_R1: 0}))
        self.next_trot_toggle = rospy.Time.now() + rospy.Duration(1.5)

    # ---------- main loop ----------
    def tick(self, _evt):
        now = rospy.Time.now()

        joy_active = (
            self.last_human_joy is not None
            and (now - self.last_human_active_t).to_sec()
            < self.joy_active_hold
        )

        # --- Joystick has priority ---
        if joy_active:
            # Pass joystick through verbatim.
            self.joy_pub.publish(self.last_human_joy)
            # Tell AI side we are overridden, so vision stops sending cmds.
            self._publish_enable(False)
            # Reset trot bookkeeping; human is in charge of gait now.
            self.trot_active = False
            return

        # --- Joystick idle: re-enable AI if user wants it ---
        self._publish_enable(self.enabled)
        if not self.enabled:
            self.joy_pub.publish(self._make_joy())
            self.trot_active = False
            return

        stale = (now - self.last_cmd_t).to_sec() > 0.5

        # --- IDLE / stale: zero everything and drop out of trot ---
        if self.mode == "IDLE" or stale:
            if self.trot_active and now > self.next_trot_toggle:
                self._trot_toggle_pulse()
                self.trot_active = False
            else:
                self.joy_pub.publish(self._make_joy())
            return

        # --- Gait management: only FOLLOW walks (TROT); others stay REST ---
        if self.mode == "FOLLOW":
            if not self.trot_active and now > self.next_trot_toggle:
                self._trot_toggle_pulse()
                self.trot_active = True
                return
        else:
            # DANCE and any non-FOLLOW active mode must stay in REST.
            if self.trot_active and now > self.next_trot_toggle:
                self._trot_toggle_pulse()
                self.trot_active = False
                return

        # --- Normal AI command -> Joy ---
        fwd = self.clamp(self.cur_cmd.linear.x / self.max_x, -1.0, 1.0)
        yaw = self.clamp(self.cur_cmd.angular.z / self.max_yaw, -1.0, 1.0)

        if self.mode == "DANCE":
            # DANCE: no translation at all. Left stick forced to zero.
            # Only right stick axes drive in-place head/body motion.
            head_yaw = self.clamp(self.cur_cmd.angular.x, -0.6, 0.6)
            body_wiggle = self.clamp(self.cur_cmd.angular.y, -0.6, 0.6)
            self.joy_pub.publish(self._make_joy(
                axes_dict={
                    AX_LX: 0.0,          # force no strafing
                    AX_LY: 0.0,          # force no forward/backward
                    AX_RX: head_yaw,     # head yaw (right stick horizontal)
                    AX_RY: body_wiggle,  # body wiggle (right stick vertical)
                }))
        else:  # FOLLOW or others
            self.joy_pub.publish(self._make_joy(
                axes_dict={AX_LY: fwd, AX_RX: yaw}))


if __name__ == "__main__":
    AIBridge()
    rospy.spin()
