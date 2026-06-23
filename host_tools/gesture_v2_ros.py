#!/usr/bin/env python3
"""
gesture_v2_ros.py

Dingo Gesture (YOLOv8-pose) -> ROS bridge via roslibpy.

Body-level gestures (no MediaPipe):
    * Both wrists above shoulders (1s) -> FOLLOW (lock closest person)
    * One wrist above head        (1s) -> DANCE
    * Both wrists below hips      (1s) -> IDLE / STOP

Publishes via rosbridge (ws://AI_ROS_HOST:AI_ROS_PORT):
    /ai_camera/cmd     geometry_msgs/Twist
    /ai_camera/mode    std_msgs/String
Subscribes:
    /ai_camera/enable  std_msgs/Bool   (toggle AI takeover at runtime)
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

TARGET_BBOX_RATIO = 0.35
DEAD_ZONE_X = 0.10
DEAD_ZONE_D = 0.05

TRAIL_LEN = 80
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
    "cmd": {"yaw": 0.0, "fwd": 0.0, "head_yaw": 0.0, "wiggle": 0.0},
    "fps": 0.0,
    "locked": False,
    "info": "",
    "persons": 0,
    "ros_connected": False,
    "ros_enabled": True,
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
    if d is None:
        return "NONE", None

    lsho = kp(d, KP_LSHO)
    rsho = kp(d, KP_RSHO)
    lwri = kp(d, KP_LWRI)
    rwri = kp(d, KP_RWRI)
    nose = kp(d, KP_NOSE)
    lhip = kp(d, KP_LHIP)
    rhip = kp(d, KP_RHIP)

    sho_y = avg_y(lsho, rsho)
    hip_y = avg_y(lhip, rhip)
    head_y = nose[1] if nose is not None else None

    if sho_y is None or (lwri is None and rwri is None):
        return "NONE", None

    if lwri is not None and rwri is not None:
        wrist_pt = lwri if lwri[1] < rwri[1] else rwri
    else:
        wrist_pt = lwri if lwri is not None else rwri

    left_up = lwri is not None and lwri[1] < sho_y
    right_up = rwri is not None and rwri[1] < sho_y

    one_above_head = False
    if head_y is not None:
        if lwri is not None and lwri[1] < head_y:
            one_above_head = True
        if rwri is not None and rwri[1] < head_y:
            one_above_head = True

    both_down = False
    if hip_y is not None:
        l_down = lwri is None or lwri[1] > hip_y
        r_down = rwri is None or rwri[1] > hip_y
        both_down = l_down and r_down

    if left_up and right_up:
        return "BOTH_UP", wrist_pt
    if one_above_head and not (left_up and right_up):
        return "ONE_UP", wrist_pt
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
# Dance Signal
# =========================
def calc_dance_wiggle(trail):
    if len(trail) < 8:
        return 0.0
    pts = [p for p, _ in list(trail)[-18:]]
    cx = float(np.mean([p[0] for p in pts]))
    cy = float(np.mean([p[1] for p in pts]))
    radius = np.mean([math.hypot(p[0] - cx, p[1] - cy) for p in pts])
    if radius < 12:
        return 0.0
    angles = [math.atan2(p[1] - cy, p[0] - cx) for p in pts]
    total = 0.0
    for i in range(1, len(angles)):
        da = angles[i] - angles[i - 1]
        da = (da + math.pi) % (2 * math.pi) - math.pi
        total += da
    if abs(total) < 0.18:
        return 0.0
    return clamp(total * 0.45, -1.0, 1.0)


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
            self.enable_topic.subscribe(self._on_enable)
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
                    "linear": {"x": cmd["fwd"], "y": 0.0, "z": 0.0},
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
            zero = {"yaw": 0.0, "fwd": 0.0,
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

    finger_trail = collections.deque(maxlen=TRAIL_LEN)
    wrist_smooth = None

    detections_cache = []
    frame_idx = 0
    mode_switch_time = 0.0

    fps_ema = 0.0
    t_prev = time.time()

    print(f"[ready] web debug: http://<pi-ip>:{WEB_PORT}/")

    while True:
        raw = picam2.capture_array()
        # Picamera2 RGB888 already returns RGB ordering
        rgb = raw

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

        if triggered and not in_cooldown:
            if triggered == "BOTH_UP" and mode != "FOLLOW":
                if closest is not None:
                    tracker.lock_box(closest["box"])
                    mode = "FOLLOW"
                    finger_trail.clear()
                    latch.reset()
                    mode_switch_time = time.time()
                    info = "FOLLOW: target locked"
                else:
                    info = "FOLLOW failed: no person"

            elif triggered == "ONE_UP" and mode != "DANCE":
                mode = "DANCE"
                tracker.clear()
                finger_trail.clear()
                latch.reset()
                mode_switch_time = time.time()
                info = "DANCE mode"

            elif triggered == "DOWN" and mode != "IDLE":
                mode = "IDLE"
                tracker.clear()
                finger_trail.clear()
                latch.reset()
                mode_switch_time = time.time()
                info = "STOP -> IDLE"

        cmd = {"yaw": 0.0, "fwd": 0.0, "head_yaw": 0.0, "wiggle": 0.0}

        if mode == "FOLLOW":
            box = tracker.update(detections)
            if tracker.is_lost():
                mode = "IDLE"
                tracker.clear()
                latch.reset()
                info = "target lost -> IDLE"
            elif box is not None:
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2
                bh = max(1, y2 - y1)
                err_x = (cx - W / 2) / (W / 2)
                ratio = bh / H
                err_d = TARGET_BBOX_RATIO - ratio
                yaw = -1.0 * err_x if abs(err_x) > DEAD_ZONE_X else 0.0
                fwd = 2.0 * err_d if abs(err_d) > DEAD_ZONE_D else 0.0
                cmd["yaw"] = clamp(yaw, -0.6, 0.6)
                cmd["fwd"] = clamp(fwd, -0.4, 0.5)

        elif mode == "DANCE":
            if wrist_pt is not None:
                finger_trail.append((wrist_pt, time.time()))
                cmd["head_yaw"] = clamp(
                    (wrist_pt[0] - W / 2) / (W / 2), -1.0, 1.0)
            cmd["wiggle"] = calc_dance_wiggle(finger_trail)

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

        if wrist_pt:
            cv2.circle(vis, wrist_pt, 8, (0, 255, 255), -1)

        if mode == "DANCE":
            pts = [p for p, _ in finger_trail]
            for i in range(1, len(pts)):
                alpha = i / max(1, len(pts) - 1)
                color = (0, int(180 + 75 * alpha), 255)
                cv2.line(vis, pts[i - 1], pts[i], color, 2)

        cv2.line(vis, (W // 2, 0), (W // 2, H), (60, 60, 60), 1)

        if hold_progress > 0:
            bar_w = int(W * hold_progress)
            cv2.rectangle(vis, (0, 38), (bar_w, 44), (0, 220, 0), -1)

        cv2.rectangle(vis, (0, 0), (W, 38), (0, 0, 0), -1)
        cd_txt = " [COOLDOWN]" if in_cooldown else ""
        ai_txt = "AI:ON" if ros_enabled else "AI:OFF"
        ros_txt = "ROS:OK" if ros.is_connected else "ROS:--"
        cv2.putText(
            vis,
            f"MODE:{mode}{cd_txt}  G:{gesture_now}  "
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


# =========================
# Flask Web Server (debug only)
# =========================
app = Flask(__name__)

PAGE = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Dingo Gesture (Pose+ROS)</title>
<style>
body{font-family:monospace;background:#111;color:#eee;margin:20px}
img{border:2px solid #444;width:960px;height:720px;
    image-rendering:pixelated}
.row{display:flex;gap:28px;align-items:flex-start}
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
  <img src="/stream.mjpg">
  <div class="s" id="s">loading...</div>
</div>

<p>
<button onclick="fetch('/ai/on')">AI ON</button>
<button onclick="fetch('/ai/off')">AI OFF</button>
</p>

<p class="small">
Gestures (hold 1s):<br>
- Both wrists above shoulders -> FOLLOW<br>
- One wrist above head -> DANCE<br>
- Both wrists below hips -> IDLE / STOP<br>
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
    return render_template_string(PAGE)


@app.route("/stat")
def stat():
    with share_lock:
        return jsonify({
            k: share[k] for k in (
                "mode", "gesture", "raw_gesture", "hold_progress",
                "cmd", "fps", "locked", "info", "persons",
                "ros_connected", "ros_enabled",
            )
        })


@app.route("/ai/on")
def ai_on():
    with share_lock:
        share["ros_enabled"] = True
    return "AI ON"


@app.route("/ai/off")
def ai_off():
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
