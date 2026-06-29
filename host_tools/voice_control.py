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

TROT_DRIFT_BIAS = 0.1
KEEPALIVE_AXIS  = 6
KEEPALIVE_VAL   = 0.20
TROT_YAW_BIAS   = 0.05

# axes[5] = body height (integrated rate); negative raises, positive lowers
HEIGHT_AXIS = 5

_axes = [0.0] * NUM_AXES
_buttons = [0] * NUM_BUTTONS
_lock = threading.Lock()
_client = None
_topic = None
_enable_topic = None
_last_seen = time.time()
_watchdog_armed = False
_trot_active = False


def _publisher_loop():
    global _trot_active
    while True:
        if _watchdog_armed and (time.time() - _last_seen) > WATCHDOG_TIMEOUT:
            with _lock:
                for i in range(NUM_AXES):
                    _axes[i] = 0.0
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
    """Wait for rosbridge + ROS stack to settle, then tap L1 to enter standby."""
    time.sleep(2.0)
    while _client is None or not _client.is_connected:
        time.sleep(0.5)
    time.sleep(1.5)   # extra settle time for ROS nodes to be ready
    print("[robot_control] auto-activating joystick control (L1) → standby mode")
    _tap_button(4)


if ROS_AVAILABLE:
    try:
        _client = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
        _client.run()
        _topic = roslibpy.Topic(_client, JOY_TOPIC, "sensor_msgs/Joy")
        _enable_topic = roslibpy.Topic(_client, "/ai_camera/enable", "std_msgs/Bool")
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


def halt():
    """Zero all motion axes without changing trot state. Use on d-pad release."""
    _set_axis(None, 0.0)


def activate():
    print(">>> ROBOT: enabling joystick control (L1)")
    stop()
    time.sleep(0.20)
    _tap_button(4)
    feed_watchdog()


def toggle_trot():
    global _trot_active, _watchdog_armed
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
    _set_axis(1, 1.0)


def backward():
    print(">>> ROBOT: MOVING BACKWARD")
    _set_axis(1, -1.0)


def left():
    print(">>> ROBOT: TURNING LEFT")
    _set_axis(3, 1.0)


def right():
    print(">>> ROBOT: TURNING RIGHT")
    _set_axis(3, -1.0)


def set_stick(stick, x, y):
    """Analog twin-stick control (x,y each in [-1,1]).
    left  = translate: axes[0] strafe, axes[1] forward/back
    right = axes[3] yaw/turn, axes[4] pitch
    Only touches its own axes — the two sticks are independent and neither
    disturbs trot state. Translation (left stick) needs trot mode to move."""
    with _lock:
        if stick == "left":
            _axes[0] = -x    # strafe  (flip sign if it strafes the wrong way)
            _axes[1] = y     # forward/back (matches d-pad forward = +)
        elif stick == "right":
            _axes[3] = -x    # yaw/turn (matches d-pad left = +axes[3])
            _axes[4] = y     # pitch   (flip sign if it tilts the wrong way)


def hop():
    """Hop request — buttons[0] (X on PS4). Firmware note: listed as unimplemented."""
    print(">>> ROBOT: HOP")
    _tap_button(0)


def height_up():
    """Raise body. Hold to keep rising, release to hold at new height."""
    print(">>> ROBOT: HEIGHT UP")
    _set_axis(HEIGHT_AXIS, -1.0)   # negative = raise (per InputInterface sign)


def height_down():
    """Lower body. Hold to keep lowering, release to hold at new height."""
    print(">>> ROBOT: HEIGHT DOWN")
    _set_axis(HEIGHT_AXIS, 1.0)    # positive = lower


def stop():
    """Emergency stop: zero all axes and clear trot state."""
    print(">>> ROBOT: STOPPED")
    global _trot_active
    with _lock:
        for i in range(NUM_AXES):
            _axes[i] = 0.0
    _trot_active = False


def take_control():
    print(">>> JOYSTICK: taking control back (override OFF)")
    if _enable_topic is not None:
        _enable_topic.publish(roslibpy.Message({"data": False}))
    stop()
