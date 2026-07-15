#!/usr/bin/env python3
"""
gesture_v2_ros.py

Dingo Gesture (YOLOv8-pose) -> ROS bridge via roslibpy.

Body-level gestures (no MediaPipe):
    * Both wrists below hips (1s) -> IDLE / STOP
      (the old both-wrists-above-shoulders / one-wrist-above-head gestures
      that used to trigger FOLLOW / DANCE are gone now that the phone
      controller has dedicated FOLLOW / DANCE / LINE buttons for that --
      see mode_select below.)

External triggers (no gesture needed):
    * /ai_camera/trigger_capture == True -> wrist-trace capture-then-replay:
      track the wrist for DRAW_WAIT_SEC, then replay that traced shape as
      right-stick deflections over DRAW_EXEC_SEC. Used by the phone
      controller's LINE button.
    * /ai_camera/mode_select == "FOLLOW" -> locks onto and continuously
      follows the closest person.
      == "DANCE" -> runs the built-in choreographed dance (right-stick
      circle for DRAW_EXEC_SEC, then back to IDLE).
      == "IDLE" -> immediately cancels FOLLOW/DRAW_WAIT/DRAW_EXEC and drops
      to IDLE. Used by the phone controller's FOLLOW / DANCE / AI-OFF
      buttons. A new FOLLOW/DANCE/LINE request always interrupts an
      in-progress DRAW_WAIT/DRAW_EXEC rather than queuing behind it.

Publishes via rosbridge (ws://AI_ROS_HOST:AI_ROS_PORT):
    /ai_camera/cmd     geometry_msgs/Twist
    /ai_camera/mode    std_msgs/String
Subscribes:
    /ai_camera/enable          std_msgs/Bool   (toggle AI takeover at runtime)
    /ai_camera/trigger_capture std_msgs/Bool   (external trigger -> start DRAW)
    /ai_camera/mode_select     std_msgs/String (external trigger -> force FOLLOW/DANCE)
"""

import os
import time
import math
import atexit
import signal
import threading
import collections

# Use all 4 Pi 5 cores for torch/openblas
os.environ["OMP_NUM_THREADS"] = "4"
os.environ["MKL_NUM_THREADS"] = "4"
os.environ["OPENBLAS_NUM_THREADS"] = "4"

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template_string
from picamera2 import Picamera2
from ultralytics import YOLO

import torch
torch.set_num_threads(4)

import roslibpy

# =========================
# Basic Config
# =========================
W, H = 640, 480

POSE_MODEL_PATH = os.environ.get(
    "POSE_MODEL_PATH",
    "/home/htt/models/yolov8n-pose.pt",
)
POSE_CONF = 0.35
POSE_IMG_SIZE = 256
POSE_EVERY_N_FRAMES = 2

GESTURE_HOLD_SEC = 1.0
LOST_TIMEOUT_SEC = 1.5
MODE_SWITCH_COOLDOWN = 1.2

# FOLLOW distance control uses a band instead of a single point target.
# If the person's bounding box ratio is below the lower bound, move forward.
# If it is above the upper bound, move backward.
FOLLOW_BBOX_RATIO_LOW = 0.55
FOLLOW_BBOX_RATIO_HIGH = 0.75
DEAD_ZONE_X = 0.10
FOLLOW_FWD_GAIN = 2.0

# Trail buffer length shared by DRAW's wrist-trace visualization. Sized for
# DRAW_WAIT_SEC seconds of capture at up to 60 samples/sec.
DRAW_TRAIL_LEN = 600

# DANCE is a built-in choreographed move: the robot sweeps its RIGHT STICK
# through a smooth circle for DRAW_EXEC_SEC seconds, so the body continuously
# sways left/right (yaw, axis 3) AND tilts up/down (pitch, axis 4) -- exactly
# like circling the standby right stick. It stays on its spot (no forward
# gait) and reliably produces both axes for the full duration, unlike the
# older wrist-trace replay which depended on what the camera happened to
# capture and often came out short or yaw-only.
#
# HOP (external trigger_capture) and the phone's LINE button both use the
# wrist-trace capture-then-replay path below: track the wrist for
# DRAW_WAIT_SEC, then replay that traced shape as the same right-stick
# deflections over DRAW_EXEC_SEC.
DRAW_WAIT_SEC = 10.0    # wrist-trace capture window (LINE button + HOP)
DRAW_EXEC_SEC = 10.0    # dance / replay length (was 5.0 -- dance longer)
# 2026-07-14: 0.7 was previously assumed safe (it's what DANCE used), but
# DANCE at 0.7 turned out to ALSO stretch the legs far enough to freeze the
# robot and require a reset -- so 0.7 was never actually safe, it just hadn't
# been pushed hard enough to show it yet. Cut hard now that we know better:
# PITCH gets the biggest cut because DANCE's yaw/pitch are 90 degrees out of
# phase (see dance_choreography below), so the moment pitch is at its peak,
# yaw is passing through ~0 -- meaning pitch alone, at its peak value, is
# what stretched the legs. Still unverified on hardware; start conservative.
DRAW_TURN_GAIN = 0.5    # wrist horizontal extent -> yaw (axis 3), capped
DRAW_PITCH_GAIN = 0.3   # wrist vertical extent   -> pitch (axis 4), capped harder
DRAW_EASE_OUT_FRAC = 0.15  # last 15% of playback eases yaw/pitch back to 0
DRAW_SAMPLE_HZ = 20

# Built-in DANCE choreography (right-stick circle): sway/tilt extent on each
# axis and how many full circles to trace over DRAW_EXEC_SEC. Kept equal to
# LINE's DRAW_TURN_GAIN/DRAW_PITCH_GAIN above -- same physical axes, same
# risk of over-stretching the legs, so they should move together.
DANCE_YAW_AMP = DRAW_TURN_GAIN     # left/right sway extent   (axis 3)
DANCE_PITCH_AMP = DRAW_PITCH_GAIN  # up/down tilt extent      (axis 4)
DANCE_CYCLES = 3.0      # circles traced over the dance

WRIST_EMA = 0.55

# COCO keypoint indices used by YOLOv8-pose
KP_NOSE = 0
KP_LSHO = 5
KP_RSHO = 6
KP_LWRI = 9
KP_RWRI = 10
KP_LHIP = 11
KP_RHIP = 12

KP_CONF_TH = 0.30

# ROS
ROS_HOST = os.environ.get("AI_ROS_HOST", "127.0.0.1")
ROS_PORT = int(os.environ.get("AI_ROS_PORT", "9090"))
ROS_PUB_HZ = 20.0

# Flask
WEB_ENABLE = os.environ.get("AI_WEB", "1") == "1"
WEB_PORT = int(os.environ.get("AI_WEB_PORT", "8080"))


# =========================
# Shared State
# =========================
share_lock = threading.Lock()
share = {
    "jpg": None,
    "mode": "IDLE",
    "gesture": "NONE",
    "raw_gesture": "NONE",
    "hold_progress": 0.0,
    "cmd": {"yaw": 0.0, "fwd": 0.0, "pitch": 0.0, "head_yaw": 0.0, "wiggle": 0.0},
    "fps": 0.0,
    "locked": False,
    "info": "",
    "persons": 0,
    "ros_connected": False,
    "ros_enabled": True,
    "capture_requested": False,
    "draw_countdown": 0.0,
    "mode_requested": "",
}


# =========================
# Utility
# =========================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


# =========================
# Pose Detector
# =========================
class PoseDetector:
    def __init__(self, model_path):
        if not os.path.isfile(model_path):
            print(f"[warn] model file not found: {model_path}")
            print("[warn] ultralytics will try to download yolov8n-pose.pt; "
                  "if offline this will fail.")
        print(f"[init] loading YOLOv8-pose: {model_path}")
        self.model = YOLO(model_path)

    def detect(self, bgr):
        results = self.model.predict(
            bgr,
            imgsz=POSE_IMG_SIZE,
            conf=POSE_CONF,
            classes=[0],
            verbose=False,
        )
        out = []
        if not results:
            return out
        r = results[0]
        if r.boxes is None or r.keypoints is None:
            return out

        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        kps_xy = r.keypoints.xy.cpu().numpy()
        kps_cf = r.keypoints.conf
        if kps_cf is not None:
            kps_cf = kps_cf.cpu().numpy()
        else:
            kps_cf = np.ones(kps_xy.shape[:2])

        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes[i]
            kps = np.concatenate(
                [kps_xy[i], kps_cf[i][:, None]], axis=1
            )
            out.append({
                "box": (int(x1), int(y1), int(x2), int(y2),
                        float(confs[i])),
                "kps": kps,
            })
        return out


def path_to_commands(points, exec_duration=DRAW_EXEC_SEC,
                      turn_gain=DRAW_TURN_GAIN, pitch_gain=DRAW_PITCH_GAIN,
                      hz=DRAW_SAMPLE_HZ):
    """Convert a drawn screen-space path into a (t_offset, yaw, pitch) command
    sequence that replays the wrist motion in place as RIGHT-JOYSTICK
    deflections: horizontal wrist position -> yaw (axis 3, left/right),
    vertical wrist position -> pitch (axis 4, up/down). Moving your wrist in a
    shape is exactly like moving the standby right stick in that same shape --
    the body sways/tilts to follow it and never walks off its spot. Returns
    (commands, resampled_points); resampled_points is the arc-length smoothed
    path (same order/length basis as commands) for visualization.

    Points are normalized (x, y) in [0, 1], y growing downward (canvas/screen
    convention), in the order they were drawn. The path is arc-length
    resampled for smooth, evenly-timed playback, then each point's offset from
    the trace's bounding-box center is mapped to a stick deflection so the
    middle of your motion is neutral and the full drawn extent reaches full
    stick.
    """
    if not points or len(points) < 3:
        return [], []

    pts = [(float(p[0]), float(p[1])) for p in points]
    seglen = [
        math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        for i in range(len(pts) - 1)
    ]
    total = sum(seglen)
    if total < 1e-4:
        return [], []

    cum = [0.0]
    for l in seglen:
        cum.append(cum[-1] + l)

    n_samples = max(10, int(exec_duration * hz))
    resampled = []
    for k in range(n_samples + 1):
        target = total * k / n_samples
        idx = 0
        while idx < len(cum) - 2 and cum[idx + 1] < target:
            idx += 1
        seg_len = seglen[idx] if seglen[idx] > 1e-9 else 1e-9
        t = clamp((target - cum[idx]) / seg_len, 0.0, 1.0)
        x = pts[idx][0] + t * (pts[idx + 1][0] - pts[idx][0])
        y = pts[idx][1] + t * (pts[idx + 1][1] - pts[idx][1])
        resampled.append((x, y))

    # Center on the trace's bounding-box midpoint (neutral = middle of your
    # motion) and scale by its half-extent so the widest/tallest part of the
    # drawing reaches full stick deflection on each axis independently.
    xs = [p[0] for p in resampled]
    ys = [p[1] for p in resampled]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    half_x = max((max(xs) - min(xs)) / 2.0, 1e-3)
    half_y = max((max(ys) - min(ys)) / 2.0, 1e-3)

    commands = []
    n = len(resampled)
    dt = exec_duration / n
    ease_n = max(1, int(n * DRAW_EASE_OUT_FRAC))
    for i, (x, y) in enumerate(resampled):
        ox = (x - cx) / half_x          # -1..1, + = wrist to the right
        oy = (y - cy) / half_y          # -1..1, + = wrist lower on screen
        # Match the standby right stick: wrist right -> yaw right, wrist up
        # -> pitch up. Flip either sign here if a direction comes out mirrored.
        yaw = clamp(turn_gain * ox, -1.0, 1.0)
        pitch = clamp(-pitch_gain * oy, -1.0, 1.0)
        # Ease the final stretch back to neutral so playback always hands off
        # to IDLE at (yaw=0, pitch=0) instead of wherever the trace happened
        # to end -- an abrupt jump from a held deflection straight to IDLE is
        # what left the robot leaning/unbalanced after LINE.
        if i >= n - ease_n:
            w = (n - 1 - i) / float(ease_n)   # 1.0 -> 0.0 over the tail
            yaw *= w
            pitch *= w
        commands.append((i * dt, yaw, pitch))
    return commands, resampled


def dance_choreography(duration=DRAW_EXEC_SEC, hz=DRAW_SAMPLE_HZ,
                       yaw_amp=DANCE_YAW_AMP, pitch_amp=DANCE_PITCH_AMP,
                       cycles=DANCE_CYCLES):
    """Built-in dance: sweep the RIGHT STICK through a smooth circle so the
    body continuously sways left/right (yaw, axis 3) AND tilts up/down
    (pitch, axis 4) -- exactly like circling the standby right stick. Unlike
    the wrist-trace replay, this always drives BOTH axes for the full
    duration regardless of what the camera sees. Returns (commands, shape)
    in the same (t_offset, yaw, pitch) / normalized-point format as
    path_to_commands so the DRAW_EXEC executor and the on-screen trail render
    unchanged."""
    n = max(10, int(duration * hz))
    dt = duration / n
    commands, shape = [], []
    for i in range(n + 1):
        phase = 2.0 * math.pi * cycles * (i / n)
        yaw = clamp(yaw_amp * math.sin(phase), -1.0, 1.0)
        pitch = clamp(pitch_amp * math.cos(phase), -1.0, 1.0)
        commands.append((i * dt, yaw, pitch))
        # normalized screen point for the viz trail (y grows downward)
        shape.append((0.5 + 0.5 * yaw, 0.5 - 0.5 * pitch))
    return commands, shape


def find_closest(detections):
    if not detections:
        return None
    return max(detections, key=lambda d: d["box"][3] - d["box"][1])


def kp(d, idx):
    x, y, c = d["kps"][idx]
    if c < KP_CONF_TH:
        return None
    return int(x), int(y)


def avg_y(*pts):
    pts = [p for p in pts if p is not None]
    if not pts:
        return None
    return sum(p[1] for p in pts) / len(pts)


def classify_pose_gesture(d):
    """Only gesture left is the safety STOP (both wrists below hips) --
    FOLLOW and DANCE used to also be triggered by wrist gestures
    (both-wrists-above-shoulders / one-wrist-above-head), but those are
    redundant now that the phone controller has dedicated FOLLOW / DANCE /
    LINE buttons for that, so they were removed."""
    if d is None:
        return "NONE", None

    lwri = kp(d, KP_LWRI)
    rwri = kp(d, KP_RWRI)
    lhip = kp(d, KP_LHIP)
    rhip = kp(d, KP_RHIP)

    hip_y = avg_y(lhip, rhip)

    if lwri is None and rwri is None:
        return "NONE", None

    if lwri is not None and rwri is not None:
        wrist_pt = lwri if lwri[1] < rwri[1] else rwri
    else:
        wrist_pt = lwri if lwri is not None else rwri

    both_down = False
    if hip_y is not None:
        l_down = lwri is None or lwri[1] > hip_y
        r_down = rwri is None or rwri[1] > hip_y
        both_down = l_down and r_down

    if both_down:
        return "DOWN", wrist_pt
    return "NONE", wrist_pt


# =========================
# Gesture Latch
# =========================
class GestureLatch:
    def __init__(self, hold_sec):
        self.hold = hold_sec
        self.cur = "NONE"
        self.t0 = time.time()
        self.fired = False

    def reset(self):
        self.cur = "NONE"
        self.t0 = time.time()
        self.fired = False

    def feed(self, g):
        now = time.time()
        if g != self.cur:
            self.cur = g
            self.t0 = now
            self.fired = False
            return None, 0.0
        if g == "NONE" or self.fired:
            return None, 0.0
        elapsed = now - self.t0
        progress = clamp(elapsed / self.hold, 0.0, 1.0)
        if elapsed >= self.hold:
            self.fired = True
            return g, 1.0
        return None, progress


# =========================
# Person Tracker
# =========================
class PersonTracker:
    def __init__(self):
        self.locked_box = None
        self.locked_h = None
        self.last_seen = 0.0

    def clear(self):
        self.locked_box = None
        self.locked_h = None
        self.last_seen = 0.0

    def is_lost(self):
        if self.locked_box is None:
            return False
        return time.time() - self.last_seen > LOST_TIMEOUT_SEC

    def lock_box(self, box):
        x1, y1, x2, y2, _ = box
        self.locked_box = (x1, y1, x2, y2)
        self.locked_h = max(1, y2 - y1)
        self.last_seen = time.time()

    def update(self, detections):
        if self.locked_box is None or not detections:
            return None
        boxes = [d["box"] for d in detections]

        best_box, best_iou = None, 0.0
        for x1, y1, x2, y2, _ in boxes:
            box = (x1, y1, x2, y2)
            u = iou(self.locked_box, box)
            if u > best_iou:
                best_iou, best_box = u, box

        if best_box is not None and best_iou >= 0.25:
            self.locked_box = best_box
            self.locked_h = (
                0.75 * self.locked_h
                + 0.25 * max(1, best_box[3] - best_box[1])
            )
            self.last_seen = time.time()
            return best_box

        lx1, ly1, lx2, ly2 = self.locked_box
        lcx, lcy = (lx1 + lx2) / 2, (ly1 + ly2) / 2
        lh = max(1, self.locked_h)

        best_box, best_score = None, 1e9
        for x1, y1, x2, y2, conf in boxes:
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            h = max(1, y2 - y1)
            score = (
                math.hypot(cx - lcx, cy - lcy)
                + abs(h - lh) * 1.6
                - conf * 5.0
            )
            if score < best_score:
                best_score = score
                best_box = (x1, y1, x2, y2)

        if best_box is not None and best_score < lh * 0.75:
            self.locked_box = best_box
            self.locked_h = (
                0.85 * self.locked_h
                + 0.15 * max(1, best_box[3] - best_box[1])
            )
            self.last_seen = time.time()
            return best_box
        return None


# =========================
# ROS Bridge (with auto-reconnect)
# =========================
class RosBridge:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = None
        self.cmd_topic = None
        self.mode_topic = None
        self.enable_topic = None
        self.enable_pub_topic = None
        self.capture_topic = None
        self.mode_select_topic = None
        self._lock = threading.Lock()
        self._last_pub_t = 0.0
        self._stopping = False

    def start(self):
        self._connect()
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _connect(self):
        try:
            if self.client is not None:
                try:
                    self.client.terminate()
                except Exception:
                    pass
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()
            self.cmd_topic = roslibpy.Topic(
                self.client, "/ai_camera/cmd", "geometry_msgs/Twist")
            self.mode_topic = roslibpy.Topic(
                self.client, "/ai_camera/mode", "std_msgs/String")
            self.enable_topic = roslibpy.Topic(
                self.client, "/ai_camera/enable", "std_msgs/Bool")
            self.enable_pub_topic = roslibpy.Topic(
                self.client, "/ai_camera/enable", "std_msgs/Bool")
            self.capture_topic = roslibpy.Topic(
                self.client, "/ai_camera/trigger_capture", "std_msgs/Bool")
            self.mode_select_topic = roslibpy.Topic(
                self.client, "/ai_camera/mode_select", "std_msgs/String")
            self.enable_topic.subscribe(self._on_enable)
            self.capture_topic.subscribe(self._on_trigger_capture)
            self.mode_select_topic.subscribe(self._on_mode_select)
            self._publish_enable_state(share["ros_enabled"])
            print(f"[ros] connected to ws://{self.host}:{self.port}")
        except Exception as e:
            print(f"[ros] connect failed: {e}")

    def _watchdog(self):
        while not self._stopping:
            if not self.is_connected:
                print("[ros] not connected, retrying...")
                self._connect()
            time.sleep(2.0)

    def _on_enable(self, msg):
        with share_lock:
            share["ros_enabled"] = bool(msg["data"])
        print(f"[ros] enable={share['ros_enabled']}")

    def _on_trigger_capture(self, msg):
        if msg.get("data"):
            with share_lock:
                share["capture_requested"] = True
            print("[ros] capture triggered (external)")

    def _on_mode_select(self, msg):
        mode = (msg.get("data") or "").strip().upper()
        if mode in ("FOLLOW", "DANCE", "IDLE"):
            with share_lock:
                share["mode_requested"] = mode
            print(f"[ros] mode requested (external): {mode}")

    def _publish_enable_state(self, enabled):
        if not self.is_connected or self.enable_pub_topic is None:
            return
        message = roslibpy.Message({"data": bool(enabled)})
        for _ in range(3):
            try:
                self.enable_pub_topic.publish(message)
            except Exception as e:
                print(f"[ros] enable publish error: {e}")
                return
            time.sleep(0.05)

    def set_enable(self, enabled):
        enabled = bool(enabled)
        with share_lock:
            share["ros_enabled"] = enabled
        self._publish_enable_state(enabled)

    @property
    def is_connected(self):
        return self.client is not None and self.client.is_connected

    def publish(self, cmd, mode, enabled, force=False):
        now = time.time()
        if not force and now - self._last_pub_t < 1.0 / ROS_PUB_HZ:
            return
        self._last_pub_t = now

        if not self.is_connected:
            return

        try:
            if enabled:
                twist = {
                    "linear": {"x": cmd["fwd"], "y": cmd["pitch"], "z": 0.0},
                    "angular": {
                        "x": cmd["head_yaw"],
                        "y": cmd["wiggle"],
                        "z": cmd["yaw"],
                    },
                }
                self.cmd_topic.publish(roslibpy.Message(twist))
                self.mode_topic.publish(
                    roslibpy.Message({"data": mode}))
            else:
                zero = {
                    "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
                self.cmd_topic.publish(roslibpy.Message(zero))
                self.mode_topic.publish(
                    roslibpy.Message({"data": "OFF"}))
        except Exception as e:
            print(f"[ros] publish error: {e}")

    def safe_stop(self):
        try:
            zero = {"yaw": 0.0, "fwd": 0.0, "pitch": 0.0,
                    "head_yaw": 0.0, "wiggle": 0.0}
            for _ in range(3):
                self.publish(zero, "OFF", True, force=True)
                time.sleep(0.05)
            print("[ros] safe stop sent.")
        except Exception as e:
            print(f"[ros] safe_stop error: {e}")

    def shutdown(self):
        self._stopping = True
        try:
            self.safe_stop()
        finally:
            try:
                if self.client is not None:
                    self.client.terminate()
            except Exception:
                pass


# =========================
# Vision Loop
# =========================
_ros_singleton = None


def vision_loop():
    global _ros_singleton

    ros = RosBridge(ROS_HOST, ROS_PORT)
    ros.start()
    _ros_singleton = ros

    detector = PoseDetector(POSE_MODEL_PATH)

    print("[init] starting Picamera2...")
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        buffer_count=4,
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(1.0)

    mode = "IDLE"
    tracker = PersonTracker()
    latch = GestureLatch(GESTURE_HOLD_SEC)

    wrist_smooth = None

    draw_wait_start = 0.0
    draw_exec_start = 0.0
    draw_commands = []
    draw_shape_pts = []
    draw_exec_idx = 0
    draw_wrist_trail = collections.deque(maxlen=DRAW_TRAIL_LEN)
    # Which wrist (L/R) LINE is tracking for the current capture, and its own
    # EMA state -- see start_capture()'s docstring for why this is separate
    # from wrist_smooth.
    draw_lock_side = None
    draw_wrist_smooth = None

    detections_cache = []
    frame_idx = 0
    mode_switch_time = 0.0

    fps_ema = 0.0
    t_prev = time.time()

    def start_capture(reason):
        """Enter DRAW_WAIT: stay still, track the wrist for DRAW_WAIT_SEC,
        then DRAW_EXEC replays the traced path as movement. Used by the
        phone's LINE button (trigger_capture) -- the "show it a motion, then
        it performs that motion" feature. DANCE (button + gesture) uses
        start_dance() instead, which plays a built-in choreography with no
        capture step.

        draw_lock_side picks ONE wrist (left or right) the first time it
        sees a hand this capture, then sticks with that same side for the
        rest of DRAW_WAIT -- unlike the general-purpose wrist_pt (used for
        the debug marker / STOP gesture), which re-picks "whichever wrist is
        currently higher" every single frame. That per-frame reselection is
        what let the trace jump to your other (resting) hand whenever the
        hand you were actually drawing with dipped low enough, which showed
        up as "the line stays in the upper part" even after your real hand
        had moved down."""
        nonlocal mode, draw_wait_start, draw_shape_pts, mode_switch_time, info
        nonlocal draw_lock_side, draw_wrist_smooth
        mode = "DRAW_WAIT"
        draw_wait_start = time.time()
        draw_wrist_trail.clear()
        draw_shape_pts = []
        draw_lock_side = None
        draw_wrist_smooth = None
        tracker.clear()
        latch.reset()
        mode_switch_time = time.time()
        info = reason

    def start_dance(reason):
        """Enter the built-in choreographed dance immediately (no wrist-trace
        capture step): DRAW_EXEC replays a right-stick circle for
        DRAW_EXEC_SEC, giving reliable left/right + up/down motion, then drops
        back to IDLE. Shared by the phone DANCE button and the one-wrist-
        above-head gesture."""
        nonlocal mode, draw_exec_start, draw_commands, draw_shape_pts
        nonlocal draw_exec_idx, mode_switch_time, info
        draw_commands, draw_shape_pts = dance_choreography()
        draw_exec_start = time.time()
        draw_exec_idx = 0
        mode = "DRAW_EXEC"
        tracker.clear()
        latch.reset()
        mode_switch_time = time.time()
        info = reason

    print(f"[ready] web debug: http://<pi-ip>:{WEB_PORT}/")

    while True:
        raw = picam2.capture_array()
        # Picamera2 RGB888 already returns RGB ordering
        rgb = cv2.flip(raw, -1)

        if frame_idx % POSE_EVERY_N_FRAMES == 0:
            detections_cache = detector.detect(rgb)
        detections = detections_cache
        frame_idx = (frame_idx + 1) % 1_000_000

        closest = find_closest(detections)
        raw_gesture, wrist_raw = classify_pose_gesture(closest)

        if wrist_raw is not None:
            if wrist_smooth is None:
                wrist_smooth = wrist_raw
            else:
                wrist_smooth = (
                    int(WRIST_EMA * wrist_raw[0]
                        + (1 - WRIST_EMA) * wrist_smooth[0]),
                    int(WRIST_EMA * wrist_raw[1]
                        + (1 - WRIST_EMA) * wrist_smooth[1]),
                )
        else:
            wrist_smooth = None

        wrist_pt = wrist_smooth
        gesture_now = raw_gesture

        triggered, hold_progress = latch.feed(gesture_now)
        info = ""

        in_cooldown = (
            time.time() - mode_switch_time < MODE_SWITCH_COOLDOWN
        )

        with share_lock:
            capture_req = share["capture_requested"]
            share["capture_requested"] = False
            mode_req = share["mode_requested"]
            share["mode_requested"] = ""

        # Explicit requests from the phone controller (LINE capture, or a
        # FOLLOW/DANCE/IDLE button) always take priority over an in-progress
        # DRAW_WAIT/DRAW_EXEC -- a button press is a deliberate command, so
        # pressing LINE or DANCE again while the previous one is still
        # running interrupts it and starts immediately instead of either
        # being silently dropped (the old capture_requested bug -- it was
        # cleared above unconditionally even when the DRAW_WAIT/DRAW_EXEC
        # guard blocked it, unlike mode_requested) or silently queuing behind
        # up to DRAW_WAIT_SEC + DRAW_EXEC_SEC of a lingering earlier request
        # (which reads to the user as "the button doesn't work").
        if (capture_req or mode_req) and mode in ("DRAW_WAIT", "DRAW_EXEC"):
            mode = "IDLE"
            tracker.clear()
            latch.reset()
            mode_switch_time = time.time()
            info = "interrupted by new request"

        if capture_req:
            start_capture("DRAW: tracing your wrist")

        if mode_req == "FOLLOW":
            if closest is not None:
                tracker.lock_box(closest["box"])
                mode = "FOLLOW"
                latch.reset()
                mode_switch_time = time.time()
                info = "FOLLOW: target locked (button)"
            else:
                info = "FOLLOW failed: no person"
        elif mode_req == "DANCE":
            start_dance("DANCE (button)")
        elif mode_req == "IDLE":
            # Sent by voice_control.py's take_control() (phone's AI OFF
            # button) so "AI off" means the camera AI actually stops now,
            # instead of a mid-flight DRAW_WAIT/DRAW_EXEC quietly finishing
            # its remaining up-to-20s in the background.
            mode = "IDLE"
            tracker.clear()
            latch.reset()
            mode_switch_time = time.time()
            info = "AI OFF -> IDLE"

        if (
            triggered
            and not in_cooldown
            and mode not in ("DRAW_WAIT", "DRAW_EXEC")
        ):
            if triggered == "DOWN" and mode != "IDLE":
                mode = "IDLE"
                tracker.clear()
                latch.reset()
                mode_switch_time = time.time()
                info = "STOP -> IDLE"

        cmd = {"yaw": 0.0, "fwd": 0.0, "pitch": 0.0, "head_yaw": 0.0, "wiggle": 0.0}

        if mode == "FOLLOW":
            # Continuous following -- no timed capture/display. Keeps
            # walking toward the locked target until it's lost or the
            # mode is switched away (STOP gesture, button, or AI-OFF).
            box = tracker.update(detections)
            if tracker.is_lost():
                mode = "IDLE"
                tracker.clear()
                latch.reset()
                info = "target lost -> IDLE"
            elif box is not None:
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                bh = max(1, y2 - y1)
                err_x = (cx - W / 2) / (W / 2)
                ratio = bh / H
                yaw = -1.0 * err_x if abs(err_x) > DEAD_ZONE_X else 0.0
                if ratio < FOLLOW_BBOX_RATIO_LOW:
                    fwd = clamp(
                        FOLLOW_FWD_GAIN * (FOLLOW_BBOX_RATIO_LOW - ratio),
                        0.0, 0.5,
                    )
                elif ratio > FOLLOW_BBOX_RATIO_HIGH:
                    fwd = clamp(
                        -FOLLOW_FWD_GAIN * (ratio - FOLLOW_BBOX_RATIO_HIGH),
                        -0.4, 0.0,
                    )
                else:
                    fwd = 0.0
                cmd["yaw"] = clamp(yaw, -0.6, 0.6)
                cmd["fwd"] = clamp(fwd, -0.4, 0.5)

        elif mode == "DRAW_WAIT":
            # Robot stays still here -- this window only captures your
            # wrist motion, it never drives off of it in real time.
            # Track ONE locked-in wrist for the whole capture (see
            # start_capture()'s docstring) instead of the general wrist_pt,
            # which re-picks "whichever wrist is currently higher" every
            # frame and would jump to your other hand mid-gesture.
            lwri_now = kp(closest, KP_LWRI) if closest is not None else None
            rwri_now = kp(closest, KP_RWRI) if closest is not None else None
            if draw_lock_side is None:
                if lwri_now is not None and rwri_now is not None:
                    draw_lock_side = "L" if lwri_now[1] < rwri_now[1] else "R"
                elif lwri_now is not None:
                    draw_lock_side = "L"
                elif rwri_now is not None:
                    draw_lock_side = "R"

            locked_raw = (
                lwri_now if draw_lock_side == "L"
                else rwri_now if draw_lock_side == "R"
                else None
            )
            if locked_raw is not None:
                if draw_wrist_smooth is None:
                    draw_wrist_smooth = locked_raw
                else:
                    draw_wrist_smooth = (
                        int(WRIST_EMA * locked_raw[0]
                            + (1 - WRIST_EMA) * draw_wrist_smooth[0]),
                        int(WRIST_EMA * locked_raw[1]
                            + (1 - WRIST_EMA) * draw_wrist_smooth[1]),
                    )
                draw_wrist_trail.append((draw_wrist_smooth, time.time()))

            if time.time() - draw_wait_start >= DRAW_WAIT_SEC:
                trace_pts = [
                    (px / W, py / H) for (px, py), _ in draw_wrist_trail
                ]
                draw_commands, draw_shape_pts = path_to_commands(trace_pts)
                if draw_commands:
                    draw_exec_start = time.time()
                    draw_exec_idx = 0
                    mode = "DRAW_EXEC"
                    mode_switch_time = time.time()
                    info = "DRAW: trace captured -> executing"
                else:
                    mode = "IDLE"
                    info = "DRAW: trace too short -> IDLE"

        elif mode == "DRAW_EXEC":
            elapsed = time.time() - draw_exec_start
            if not draw_commands or elapsed >= DRAW_EXEC_SEC:
                mode = "IDLE"
                info = "DRAW: path complete -> IDLE"
            else:
                draw_exec_idx = min(
                    int(elapsed / DRAW_EXEC_SEC * len(draw_commands)),
                    len(draw_commands) - 1,
                )
                _, yaw, pitch = draw_commands[draw_exec_idx]
                cmd["yaw"] = yaw
                cmd["fwd"] = 0.0     # never walk off the spot during dance
                cmd["pitch"] = pitch

        # publish to ROS
        with share_lock:
            ros_enabled = share["ros_enabled"]
        ros.publish(cmd, mode, ros_enabled)

        # visualization
        vis = rgb.copy()

        for d in detections:
            x1, y1, x2, y2, _ = d["box"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (120, 120, 120), 1)
            for idx in (KP_LSHO, KP_RSHO, KP_LWRI, KP_RWRI,
                        KP_NOSE, KP_LHIP, KP_RHIP):
                x, y, c = d["kps"][idx]
                if c >= KP_CONF_TH:
                    cv2.circle(vis, (int(x), int(y)),
                               3, (200, 200, 0), -1)

        if closest is not None:
            x1, y1, x2, y2, _ = closest["box"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(vis, "closest", (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 200, 255), 1)

        if tracker.locked_box:
            x1, y1, x2, y2 = [int(v) for v in tracker.locked_box]
            color = (0, 255, 0) if not tracker.is_lost() else (0, 0, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 3)
            cv2.putText(vis, "LOCKED", (x1, max(18, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        # During DRAW_WAIT show the locked-wrist point (what's actually being
        # captured), not the general per-frame "highest wrist" wrist_pt --
        # otherwise the debug dot and the trail it's supposedly drawing could
        # visibly disagree.
        marker_pt = draw_wrist_smooth if mode == "DRAW_WAIT" else wrist_pt
        if marker_pt:
            cv2.circle(vis, marker_pt, 8, (0, 255, 255), -1)

        if mode == "DRAW_WAIT":
            pts = [p for p, _ in draw_wrist_trail]
            for i in range(1, len(pts)):
                alpha = i / max(1, len(pts) - 1)
                color = (0, int(120 + 100 * alpha), 255)
                cv2.line(vis, pts[i - 1], pts[i], color, 2)

        if mode == "DRAW_EXEC" and draw_shape_pts:
            pix = [(int(x * (W - 1)), int(y * (H - 1)))
                   for x, y in draw_shape_pts]
            for i in range(1, len(pix)):
                cv2.line(vis, pix[i - 1], pix[i], (255, 180, 0), 2)
            marker = pix[min(draw_exec_idx, len(pix) - 1)]
            cv2.circle(vis, marker, 7, (0, 255, 255), -1)

        cv2.line(vis, (W // 2, 0), (W // 2, H), (60, 60, 60), 1)

        if hold_progress > 0:
            bar_w = int(W * hold_progress)
            cv2.rectangle(vis, (0, 38), (bar_w, 44), (0, 220, 0), -1)

        cv2.rectangle(vis, (0, 0), (W, 38), (0, 0, 0), -1)
        cd_txt = " [COOLDOWN]" if in_cooldown else ""
        ai_txt = "AI:ON" if ros_enabled else "AI:OFF"
        ros_txt = "ROS:OK" if ros.is_connected else "ROS:--"
        cap_txt = ""
        draw_countdown = 0.0
        if mode == "DRAW_WAIT":
            draw_countdown = max(0.0, DRAW_WAIT_SEC - (time.time() - draw_wait_start))
            cap_txt = f"  TRACE YOUR WRIST {draw_countdown:0.1f}s"
        elif mode == "DRAW_EXEC":
            remain = max(0.0, DRAW_EXEC_SEC - (time.time() - draw_exec_start))
            cap_txt = f"  TRACING PATH {remain:0.1f}s"
        cv2.putText(
            vis,
            f"MODE:{mode}{cd_txt}{cap_txt}  G:{gesture_now}  "
            f"{ai_txt} {ros_txt}  {info}",
            (5, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48,
            (255, 255, 255), 1,
        )

        cv2.rectangle(vis, (0, H - 34), (W, H), (0, 0, 0), -1)
        cv2.putText(
            vis,
            f"yaw={cmd['yaw']:+.2f} fwd={cmd['fwd']:+.2f} "
            f"head={cmd['head_yaw']:+.2f} wiggle={cmd['wiggle']:+.2f}",
            (5, H - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48,
            (0, 255, 255), 1,
        )

        now = time.time()
        dt = now - t_prev
        t_prev = now
        if dt > 0:
            inst = 1.0 / dt
            fps_ema = (
                inst if fps_ema == 0 else 0.9 * fps_ema + 0.1 * inst
            )

        cv2.putText(vis, f"{fps_ema:.1f}fps", (W - 90, H - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (220, 220, 220), 1)

        ok, jpg = cv2.imencode(
            ".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            with share_lock:
                share["jpg"] = jpg.tobytes()
                share["mode"] = mode
                share["gesture"] = gesture_now
                share["raw_gesture"] = raw_gesture
                share["hold_progress"] = round(hold_progress, 2)
                share["cmd"] = cmd
                share["fps"] = round(fps_ema, 1)
                share["locked"] = tracker.locked_box is not None
                share["info"] = info
                share["persons"] = len(detections)
                share["ros_connected"] = ros.is_connected
                share["draw_countdown"] = round(draw_countdown, 1)


# =========================
# Flask Web Server (debug only)
# =========================
app = Flask(__name__)

PAGE = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Dingo Gesture (Pose+ROS)</title>
<style>
body{font-family:monospace;background:#111;color:#eee;margin:20px}
.row{display:flex;gap:28px;align-items:flex-start;flex-wrap:wrap}
.stream-wrap{position:relative;width:100%;max-width:960px;aspect-ratio:4/3}
img{position:absolute;left:0;top:0;width:100%;height:100%;border:2px solid #444;
    box-sizing:border-box;image-rendering:pixelated}
.s{font-size:16px;line-height:1.7;min-width:300px}
b{color:#6cf}
.small{color:#aaa;font-size:14px}
button{margin:4px;padding:6px 14px;background:#222;color:#eee;
       border:1px solid #555;cursor:pointer}
button:hover{background:#333}
</style>
</head>
<body>
<h2>Dingo Gesture Test (YOLOv8-pose + ROS)</h2>
<div class="row">
  <div class="stream-wrap">
    <img src="/stream.mjpg">
  </div>
  <div class="s" id="s">loading...</div>
</div>

<p>
<button onclick="fetch('/ai/on')">AI ON</button>
<button onclick="fetch('/ai/off')">AI OFF</button>
</p>

<p class="small">
Gestures (hold 1s):<br>
- Both wrists below hips -> IDLE / STOP<br>
- Phone LINE button -> traces your wrist/hand for 10s (robot stays still), then replays that motion<br>
- Phone FOLLOW / DANCE buttons -> force that mode directly (no gesture needed)<br>
</p>

<script>
async function poll(){
  let r=await fetch('/stat'); let j=await r.json();

  document.getElementById('s').innerHTML =
    'mode: <b>'+j.mode+'</b><br>'+
    'gesture: '+j.gesture+'<br>'+
    'raw: '+j.raw_gesture+'<br>'+
    'hold_progress: '+(j.hold_progress*100).toFixed(0)+'%<br>'+
    'locked: '+j.locked+'<br>'+
    'persons: '+j.persons+'<br>'+
    'fps: '+j.fps+'<br>'+
    'ROS connected: '+j.ros_connected+'<br>'+
    'AI enabled: '+j.ros_enabled+'<br>'+
    'info: '+j.info+'<br><br>'+
    'cmd:<br>'+
    'yaw: '+j.cmd.yaw.toFixed(2)+'<br>'+
    'fwd: '+j.cmd.fwd.toFixed(2)+'<br>'+
    'head_yaw: '+j.cmd.head_yaw.toFixed(2)+'<br>'+
    'wiggle: '+j.cmd.wiggle.toFixed(2);
}
setInterval(poll,200); poll();
</script>
</body>
</html>
"""


@app.route("/")
def index():
    resp = Response(render_template_string(PAGE), mimetype="text/html")
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    return resp


@app.route("/stat")
def stat():
    with share_lock:
        return jsonify({
            k: share[k] for k in (
                "mode", "gesture", "raw_gesture", "hold_progress",
                "cmd", "fps", "locked", "info", "persons",
                "ros_connected", "ros_enabled", "draw_countdown",
            )
        })



@app.route("/ai/on")
def ai_on():
    if _ros_singleton is not None:
        _ros_singleton.set_enable(True)
    else:
        with share_lock:
            share["ros_enabled"] = True
    return "AI ON"


@app.route("/ai/off")
def ai_off():
    if _ros_singleton is not None:
        _ros_singleton.set_enable(False)
    else:
        with share_lock:
            share["ros_enabled"] = False
    return "AI OFF"


def mjpeg_gen():
    while True:
        with share_lock:
            jpg = share["jpg"]
        if jpg is None:
            time.sleep(0.05)
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" +
            jpg + b"\r\n"
        )
        time.sleep(0.05)



@app.route("/stream.mjpg")
def stream():
    return Response(
        mjpeg_gen(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# =========================
# Safe shutdown hooks
# =========================
def _safe_stop_all():
    print("[exit] sending safe stop to ROS...")
    try:
        if _ros_singleton is not None:
            _ros_singleton.shutdown()
    except Exception as e:
        print(f"[exit] error: {e}")


atexit.register(_safe_stop_all)


def _on_signal(signum, _frame):
    print(f"[exit] got signal {signum}, shutting down...")
    _safe_stop_all()
    os._exit(0)


for _sig in (signal.SIGINT, signal.SIGTERM):
    try:
        signal.signal(_sig, _on_signal)
    except Exception:
        pass


# =========================
# Main
# =========================
if __name__ == "__main__":
    threading.Thread(target=vision_loop, daemon=True).start()

    if WEB_ENABLE:
        app.run(host="0.0.0.0", port=WEB_PORT,
                threaded=True, debug=False)
    else:
        while True:
            time.sleep(1.0)
