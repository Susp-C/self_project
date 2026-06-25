#!/usr/bin/env python3
"""
vioce_control.py — shared MOVEMENT LAYER for online_joystick.py and listen.py
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

# Deadman watchdog
WATCHDOG_TIMEOUT = 0.5

# Trot drift compensation: negative value fights natural forward creep
TROT_DRIFT_BIAS = 0.1         # ← Tuned. Increase negativity if still creeps forward
KEEPALIVE_AXIS = 6
KEEPALIVE_VAL  = 0.20
TROT_YAW_BIAS   = 0.05        # yaw/turning   (positive = left correction)

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
        # DEADMAN WATCHDOG
        if _watchdog_armed and (time.time() - _last_seen) > WATCHDOG_TIMEOUT:
            with _lock:
                for i in range(NUM_AXES):
                    _axes[i] = 0.0
            _trot_active = False

        if _client is not None and _client.is_connected:
            with _lock:
                axes_out = list(_axes)
                btns_out = list(_buttons)

            # Drift compensation when trot is active and no active user input
            if _trot_active and all(abs(a) < 0.05 for a in axes_out[:6]):
                axes_out[1] = TROT_DRIFT_BIAS   # forward/backward
                axes_out[3] = TROT_YAW_BIAS     # turning left/right
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


# ====================== ROS SETUP ======================
if ROS_AVAILABLE:
    try:
        _client = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
        _client.run()
        _topic = roslibpy.Topic(_client, JOY_TOPIC, "sensor_msgs/Joy")
        _enable_topic = roslibpy.Topic(_client, "/ai_camera/enable", "std_msgs/Bool")
        print(f"[robot_control] connected to rosbridge, publishing {JOY_TOPIC}")
        threading.Thread(target=_publisher_loop, daemon=True).start()
    except Exception as e:
        print(f"[robot_control] could not connect to rosbridge: {e}")


# ====================== INTERNAL HELPERS ======================
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
    """Reset deadman timer and arm watchdog."""
    global _last_seen, _watchdog_armed
    _last_seen = time.time()
    _watchdog_armed = True


# ====================== PUBLIC API ======================
def activate():
    """Enable joystick control (L1). Forces clean stop first."""
    print(">>> ROBOT: enabling joystick control (L1)")
    stop()                    # ← KEY FIX: Prevent immediate forward motion
    time.sleep(0.08)          # Small settle time for robot to process stop
    _tap_button(4)            # L1 = enable joystick control
    feed_watchdog()


def toggle_trot():
    global _trot_active
    _trot_active = not _trot_active
    print(f">>> ROBOT: trot {'ON — drift compensation active' if _trot_active else 'OFF'}")
    stop()                    # ← KEY FIX: Clean state before mode change
    _tap_button(5)            # R1 = toggle trot/walk


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
    _set_axis(3, -0.0)  # Note: original used -1.0 for right


def stop():
    print(">>> ROBOT: STOPPED")
    with _lock:
        for i in range(NUM_AXES):
            _axes[i] = 0.0
    # No need to call _set_axis again since we zeroed everything


def take_control():
    print(">>> JOYSTICK: taking control back (override OFF)")
    if _enable_topic is not None:
        _enable_topic.publish(roslibpy.Message({"data": False}))
    stop()
