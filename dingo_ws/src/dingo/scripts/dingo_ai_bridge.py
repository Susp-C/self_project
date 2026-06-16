#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Joy axes/buttons 映射(对照 InputInterface.py)
AX_LX = 0      # 左右平移 (y_vel)
AX_LY = 1      # 前后 (x_vel)
AX_RX = 3      # 转向 (yaw)
AX_RY = 4      # pitch
AX_DPADX = 6
AX_DPADY = 7
BTN_R1 = 5     # gait toggle (trot)
BTN_X  = 0     # hop
BTN_L1 = 4     # joystick toggle


class AIBridge:
    def __init__(self):
        rospy.init_node("dingo_ai_bridge")

        self.enabled    = rospy.get_param("~enabled", True)
        self.max_x      = rospy.get_param("~max_x",   0.4)
        self.max_y      = rospy.get_param("~max_y",   0.3)
        self.max_yaw    = rospy.get_param("~max_yaw", 1.0)
        self.cmd_rate   = rospy.get_param("~cmd_rate", 30.0)

        self.mode = "IDLE"
        self.last_cmd_t = rospy.Time.now()
        self.cur_cmd = Twist()       # 最近一次 AI 指令
        self.trot_active = False
        self.next_trot_toggle = rospy.Time.now()

        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=2)
        self.enable_pub = rospy.Publisher(
            "/ai_camera/enable", Bool, queue_size=1, latch=True)
        self.enable_pub.publish(Bool(data=self.enabled))

        rospy.Subscriber("/ai_camera/cmd",  Twist,  self.on_cmd)
        rospy.Subscriber("/ai_camera/mode", String, self.on_mode)
        rospy.Subscriber("/ai_camera/enable", Bool, self.on_enable)

        rospy.Timer(rospy.Duration(1.0 / self.cmd_rate), self.tick)
        rospy.loginfo(f"[ai_bridge] up, enabled={self.enabled}")

    def on_cmd(self, msg):
        self.last_cmd_t = rospy.Time.now()
        self.cur_cmd = msg

    def on_mode(self, msg):
        self.mode = msg.data

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        rospy.loginfo(f"[ai_bridge] enabled={self.enabled}")

    def make_joy(self, axes_dict, btn_dict=None):
        j = Joy()
        j.header.stamp = rospy.Time.now()
        j.axes = [0.0] * 8
        j.buttons = [0] * 11
        for i, v in axes_dict.items():
            j.axes[i] = float(v)
        if btn_dict:
            for i, v in btn_dict.items():
                j.buttons[i] = int(v)
        return j

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def tick(self, _):
        # AI 关闭 / IDLE / 0.5s 没收到命令 -> 全零
        stale = (rospy.Time.now() - self.last_cmd_t).to_sec() > 0.5
        if (not self.enabled) or self.mode == "IDLE" or stale:
            # 如果之前在 trot,需要再按一次 R1 让狗回到 REST
            if self.trot_active and rospy.Time.now() > self.next_trot_toggle:
                self.joy_pub.publish(self.make_joy({}, {BTN_R1: 1}))
                rospy.sleep(0.05)
                self.joy_pub.publish(self.make_joy({}, {BTN_R1: 0}))
                self.trot_active = False
                self.next_trot_toggle = rospy.Time.now() + rospy.Duration(1.5)
            else:
                self.joy_pub.publish(self.make_joy({}))
            return

        # 第一次接管 -> 按 R1 进 trot
        if not self.trot_active and rospy.Time.now() > self.next_trot_toggle:
            self.joy_pub.publish(self.make_joy({}, {BTN_R1: 1}))
            rospy.sleep(0.05)
            self.joy_pub.publish(self.make_joy({}, {BTN_R1: 0}))
            self.trot_active = True
            self.next_trot_toggle = rospy.Time.now() + rospy.Duration(1.5)
            return

        # 正常控速
        fwd = self.clamp(self.cur_cmd.linear.x  / self.max_x,   -1.0, 1.0)
        yaw = self.clamp(self.cur_cmd.angular.z / self.max_yaw, -1.0, 1.0)

        if self.mode == "DANCE":
            # head_yaw -> yaw, wiggle -> 横移
            yaw_d = self.clamp(self.cur_cmd.angular.x, -0.6, 0.6)
            lat   = self.clamp(self.cur_cmd.angular.y * 0.5, -0.6, 0.6)
            self.joy_pub.publish(self.make_joy(
                {AX_LY: 0.0, AX_LX: lat, AX_RX: yaw_d}))
        else:
            # FOLLOW
            self.joy_pub.publish(self.make_joy(
                {AX_LY: fwd, AX_RX: yaw}))


if __name__ == "__main__":
    AIBridge()
    rospy.spin()
