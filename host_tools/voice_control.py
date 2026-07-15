#!/usr/bin/env python3
"""
voice_control.py — shared MOVEMENT LAYER used by online_joystick.py
(web joystick + phone-side Web Speech API voice control).
Publishes sensor_msgs/Joy on /joy_human via rosbridge.
"""


import time
import threading

try:
    import roslibpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[robot_control] roslibpy not installed - PRINT-ONLY mode.")

ROS_HOST, ROS_PORT = "127.0.0.1", 9090
JOY_TOPIC = "/joy_human"
PUB_HZ = 20.0
NUM_AXES, NUM_BUTTONS = 8, 12

WATCHDOG_TIMEOUT = 0.5

TROT_DRIFT_BIAS = 0.0     # was 0.1 forward — caused front drift
KEEPALIVE_AXIS  = 6
KEEPALIVE_VAL   = 0.0     # was 0.20 on axes[6] (roll) — caused left drift
TROT_YAW_BIAS   = 0.0

MOVE_SCALE = 0.6          # global speed cap for D-pad and joystick (0.0-1.0)

# axes[5] = body height (integrated rate); negative lowers, positive raises
# (flipped 2026-07-14 -- confirmed reversed on the real robot vs. the dev-Pi
# sign convention this was originally written against)
HEIGHT_AXIS = 5

_axes = [0.0] * NUM_AXES
_buttons = [0] * NUM_BUTTONS
_lock = threading.Lock()
_client = None
_topic = None
_enable_topic = None
_mode_topic = None
_capture_topic = None
_last_seen = time.time()
_watchdog_armed = False
_trot_active = False
_armed = False


def _publisher_loop():
    global _trot_active
    while True:
        if _watchdog_armed and (time.time() - _last_seen) > WATCHDOG_TIMEOUT:
            with _lock:
                for i in range(NUM_AXES):
                    _axes[i] = 0.0
            if _trot_active:
                # A watchdog trip (WiFi blip, phone screen lock, backgrounded
                # tab) used to just flip this LOCAL flag to False without ever
                # telling the real robot to leave trot -- the robot kept
                # trotting while get_state() (and the UI's TROT light) said
                # OFF. That's the exact "blue is off but trot was active"
                # desync reported. Send the real toggle-off tap too, so
                # software and hardware always agree. Gated on _trot_active
                # being True *before* we clear it, so this only fires once
                # per trip, not every loop tick while tripped.
                threading.Thread(target=_tap_button, args=(5,), daemon=True).start()
            _trot_active = False

        if _client is not None and _client.is_connected:
            with _lock:
                axes_out = list(_axes)
                btns_out = list(_buttons)

            if _trot_active and all(abs(a) < 0.05 for a in axes_out[:6]):
                axes_out[1] = TROT_DRIFT_BIAS
                axes_out[3] = TROT_YAW_BIAS
                axes_out[KEEPALIVE_AXIS] = KEEPALIVE_VAL

            msg = {
                "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
                "axes": axes_out,
                "buttons": btns_out,
            }
            try:
                _topic.publish(roslibpy.Message(msg))
            except Exception as e:
                print(f"[robot_control] publish error: {e}")

        time.sleep(1.0 / PUB_HZ)


def _auto_standby():
    """The robot powers up already ARMED (standing, L1 already ON). Do NOT tap
    L1 here: L1 is a stateless toggle, so tapping it would DISARM the robot
    while we record it as armed — inverting the ENABLE button for the rest of
    the session (blue/lit = actually disarmed = trot refused, controller dead).
    We only wait for the ROS link, then record the true boot state so software
    and hardware agree. The first real toggle is the phone's force_disarm on
    connect, by which time the driver is subscribed and taps land reliably."""
    global _armed
    time.sleep(2.0)
    while _client is None or not _client.is_connected:
        time.sleep(0.5)
    time.sleep(1.5)   # let ROS nodes settle before we start publishing /joy
    print("[robot_control] robot boots ARMED (standing) → recording armed, NOT tapping L1")
    _armed = True


if ROS_AVAILABLE:
    try:
        _client = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
        _client.run()
        _topic = roslibpy.Topic(_client, JOY_TOPIC, "sensor_msgs/Joy")
        _enable_topic = roslibpy.Topic(_client, "/ai_camera/enable", "std_msgs/Bool")
        _mode_topic = roslibpy.Topic(_client, "/ai_camera/mode_select", "std_msgs/String")
        _capture_topic = roslibpy.Topic(_client, "/ai_camera/trigger_capture", "std_msgs/Bool")
        print(f"[robot_control] connected to rosbridge, publishing {JOY_TOPIC}")
        threading.Thread(target=_publisher_loop, daemon=True).start()
        threading.Thread(target=_auto_standby, daemon=True).start()
    except Exception as e:
        print(f"[robot_control] could not connect to rosbridge: {e}")


def _set_axis(index, value):
    with _lock:
        for i in range(NUM_AXES):
            _axes[i] = 0.0
        if index is not None:
            _axes[index] = value


def _tap_button(index, hold=0.15):
    with _lock:
        _buttons[index] = 1
    time.sleep(hold)
    with _lock:
        _buttons[index] = 0


def feed_watchdog():
    global _last_seen, _watchdog_armed
    _last_seen = time.time()
    _watchdog_armed = True


# ====================== PUBLIC API ======================

def is_trot_active():
    return _trot_active


def is_armed():
    return _armed


def get_state():
    """Authoritative snapshot the UI should render from — never guess client-side."""
    return {"armed": _armed, "trot": _trot_active}


def halt():
    """Zero all motion axes without changing trot state. Use on d-pad release."""
    _set_axis(None, 0.0)


def _do_arm():
    global _armed
    halt()
    time.sleep(0.20)
    _tap_button(4)
    _armed = True
    feed_watchdog()


def _do_disarm():
    global _armed, _watchdog_armed
    if _trot_active:
        toggle_trot()   # forces trot off on the robot + here, before disarming
        time.sleep(0.20)
    else:
        stop()
        time.sleep(0.20)
    _tap_button(4)
    _armed = False
    _watchdog_armed = False


def activate():
    """Toggle joystick control (L1). Disarming also forces trot off
    (robot + software), so re-arming always starts from a clean, known
    state instead of silently resuming a leftover trot."""
    if _armed:
        _do_disarm()
    else:
        _do_arm()
    print(f">>> ROBOT: {'ARMED (L1 ON)' if _armed else 'DISARMED (L1 OFF)'}")


def force_disarm():
    """Zero motion on every new phone connection WITHOUT touching hardware.
    L1 is a stateless toggle with zero feedback from the robot -- blindly
    re-tapping it here (the old behavior) based on a possibly-stale _armed
    guess could flip the REAL robot the wrong way with nothing able to
    detect it, and since reconnects happen constantly (WiFi blips, phone
    lock, page reloads -- see session.log), one bad guess compounds into a
    permanently inverted ENABLE button for the rest of the session. That is
    exactly what was reported. Arm/trot state now only ever changes via an
    explicit LB/RB press the user can see land on the physical robot, so it
    can never drift out of sync behind their back."""
    stop()


def toggle_trot():
    global _trot_active, _watchdog_armed
    if not _armed:
        # R1 tap is ignored by the robot while joystick control (L1) is off,
        # so flipping _trot_active here would desync software from hardware —
        # the next real toggle would then do the opposite of what the UI shows.
        print(">>> ROBOT: trot toggle ignored — not enabled")
        return

    intended = not _trot_active   # save before stop() resets it
    print(f">>> ROBOT: trot {'ON — drift compensation active' if intended else 'OFF'}")

    stop()
    time.sleep(0.25)

    _trot_active = intended       # restore intended state after stop()
    _tap_button(5)                # R1 = toggle trot on robot

    if _trot_active:
        feed_watchdog()
    else:
        _watchdog_armed = False


def forward():
    print(">>> ROBOT: MOVING FORWARD")
    _set_axis(1, MOVE_SCALE)


def backward():
    print(">>> ROBOT: MOVING BACKWARD")
    _set_axis(1, -MOVE_SCALE)


def left():
    print(">>> ROBOT: TURNING LEFT")
    _set_axis(3, MOVE_SCALE)


def right():
    print(">>> ROBOT: TURNING RIGHT")
    _set_axis(3, -MOVE_SCALE)


def set_stick(stick, x, y):
    """Analog twin-stick control (x,y each in [-1,1]).
    left  = trot: axes[0] strafe, axes[1] forward/back
            standby: y-axis drives HEIGHT_AXIS instead (up = raise), x-axis still strafe
    right = axes[3] yaw/turn, axes[4] pitch
    Only touches its own axes — the two sticks are independent and neither
    disturbs trot state. Translation (left stick) needs trot mode to move."""
    with _lock:
        if stick == "left":
            _axes[0] = -x * MOVE_SCALE    # strafe  (flip sign if it strafes the wrong way)
            if _trot_active:
                _axes[1] = y * MOVE_SCALE     # forward/back (matches d-pad forward = +)
            else:
                _axes[HEIGHT_AXIS] = y        # standby: up = raise (matches height_up() sign)
        elif stick == "right":
            _axes[3] = -x * MOVE_SCALE    # yaw/turn (matches d-pad left = +axes[3])
            _axes[4] = y * MOVE_SCALE     # pitch   (flip sign if it tilts the wrong way)


def hop():
    """Hop request — buttons[0] (X on PS4). Firmware note: listed as unimplemented."""
    print(">>> ROBOT: HOP")
    _tap_button(0)


def height_up():
    """Raise body. Hold to keep rising, release to hold at new height."""
    print(">>> ROBOT: HEIGHT UP")
    _set_axis(HEIGHT_AXIS, 1.0)    # positive = raise (flipped 2026-07-14, was reversed)


def height_down():
    """Lower body. Hold to keep lowering, release to hold at new height."""
    print(">>> ROBOT: HEIGHT DOWN")
    _set_axis(HEIGHT_AXIS, -1.0)   # negative = lower (flipped 2026-07-14, was reversed)


def stop():
    """Emergency stop: zero all axes and clear trot state."""
    print(">>> ROBOT: STOPPED")
    global _trot_active
    with _lock:
        for i in range(NUM_AXES):
            _axes[i] = 0.0
    _trot_active = False


def take_control():
    """Joystick takes back control (AI OFF button). Also tells the camera AI
    to drop whatever it's doing right now -- without this, a mid-flight LINE
    capture or DANCE would keep running in the background for up to
    DRAW_WAIT_SEC + DRAW_EXEC_SEC (up to ~20s) after the user asked for
    control back, which read as "AI off doesn't work"."""
    print(">>> JOYSTICK: taking control back (override OFF)")
    ai_off()
    if _mode_topic is not None:
        try:
            _mode_topic.publish(roslibpy.Message({"data": "IDLE"}))
        except Exception as e:
            print(f"[robot_control] mode publish error: {e}")
    stop()


def set_ai_enable(enabled):
    state = bool(enabled)
    print(f">>> AI CAMERA: {'ON' if state else 'OFF'}")
    if _enable_topic is None:
        return
    try:
        _enable_topic.publish(roslibpy.Message({"data": state}))
    except Exception as e:
        print(f"[robot_control] enable publish error: {e}")


def ai_on():
    set_ai_enable(True)


def ai_off():
    set_ai_enable(False)


def set_ai_mode(mode):
    """Force the camera AI directly into FOLLOW or DANCE, bypassing gesture
    detection. Also ensures AI takeover is enabled, since a mode selection
    implies the user wants the camera driving."""
    mode = (mode or "").strip().upper()
    if mode not in ("FOLLOW", "DANCE"):
        return
    print(f">>> AI CAMERA: mode -> {mode}")
    set_ai_enable(True)
    if _mode_topic is None:
        return
    try:
        _mode_topic.publish(roslibpy.Message({"data": mode}))
    except Exception as e:
        print(f"[robot_control] mode publish error: {e}")


def trigger_line_capture():
    """LINE button: camera traces your hand/finger motion for DRAW_WAIT_SEC,
    then the robot dances by replaying that traced path (gesture_v2_ros.py's
    existing capture-then-replay feature, previously only reachable via the
    HOP gesture trigger). Distinct from the DANCE button, which plays a
    fixed built-in choreography with no capture step."""
    print(">>> AI CAMERA: LINE capture starting")
    set_ai_enable(True)
    if _capture_topic is None:
        return
    try:
        _capture_topic.publish(roslibpy.Message({"data": True}))
    except Exception as e:
        print(f"[robot_control] capture publish error: {e}")

