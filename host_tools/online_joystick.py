import os
import json
import asyncio
from aiohttp import web
import time
import threading
import voice_control as robot

# ── optional voice control ──────────────────────────────────────────────────
try:
    import vosk
    import pyaudio
    VOICE_AVAILABLE = True
except ImportError:
    VOICE_AVAILABLE = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "vosk-model-small-en-us-0.15")
KEYWORDS = '["hello", "forward", "backward", "left", "right", "higher", "lower", "stop", "quit", "[unk]"]'
RATE = 48000
CHUNK = 8000
MIC_NAME = "AIMIC"
SPEAKER_DEVICE = "plughw:CARD=AIMICM4,DEV=0"


def speak(text):
    print(f"[ROBOT]: {text}")
    os.system(f'espeak "{text}" --stdout | aplay -D {SPEAKER_DEVICE} -q 2>/dev/null &')


def find_mic_index(pa):
    for i in range(pa.get_device_count()):
        d = pa.get_device_info_by_index(i)
        if d['maxInputChannels'] > 0 and MIC_NAME.lower() in d['name'].lower():
            print(f"[mic] using device {i}: {d['name']}")
            return i
    print(f"[mic] WARNING: '{MIC_NAME}' not found, falling back to default input")
    return None


def voice_loop():
    if not VOICE_AVAILABLE:
        print("[voice] vosk/pyaudio not installed — voice control disabled")
        return
    if not os.path.exists(MODEL_PATH):
        print(f"[voice] model not found at {MODEL_PATH} — voice control disabled")
        return
    try:
        model = vosk.Model(MODEL_PATH)
        rec = vosk.KaldiRecognizer(model, RATE, KEYWORDS)
        pa = pyaudio.PyAudio()
        mic_index = find_mic_index(pa)
        stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=RATE,
            input=True,
            input_device_index=mic_index,
            frames_per_buffer=CHUNK,
        )
        stream.start_stream()
    except Exception as e:
        print(f"[voice] failed to open mic: {e} — voice control disabled")
        return

    print("[voice] ready. Say 'hello' to wake the robot.")
    awake = False
    trot_active = False

    try:
        while True:
            data = stream.read(CHUNK, exception_on_overflow=False)
            if awake:
                robot.feed_watchdog()   # keep watchdog alive for all commands (movement + height)
            if not rec.AcceptWaveform(data):
                continue
            word = json.loads(rec.Result()).get("text", "").strip()
            if not word or word == "[unk]":
                continue
            print(f"[Heard]: {word}")

            if not awake:
                if word == "hello":
                    awake = True
                    speak("ready")   # auto_standby already tapped L1; don't toggle it off
            else:
                if word == "hello":
                    speak("ready")
                elif word in ("higher", "lower"):
                    speak(word)
                    if word == "higher":
                        robot.height_up()
                    else:
                        robot.height_down()
                    time.sleep(0.5)
                    robot.halt()
                elif word in ("forward", "backward", "left", "right"):
                    if not trot_active:
                        robot.toggle_trot()
                        trot_active = True
                        time.sleep(0.3)
                    speak(word)
                    getattr(robot, word)()
                elif word == "stop":
                    speak("stop")
                    if trot_active:
                        robot.toggle_trot()
                        trot_active = False
                    robot.stop()
                elif word == "quit":
                    speak("goodbye")
                    if trot_active:
                        robot.toggle_trot()
                        trot_active = False
                    robot.stop()
                    awake = False
                    print("[voice] sleeping. Say 'hello' to reactivate.")
    except Exception as e:
        print(f"[voice] error: {e}")
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()


async def index(request):
    return web.FileResponse(os.path.join(BASE_DIR, "index.html"))

async def favicon(request):
    return web.Response(status=204)

async def websocket(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    loop = asyncio.get_event_loop()
    print("Controller connected.")
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                except (ValueError, TypeError):
                    continue
                if data.get("type") == "heartbeat":
                    robot.feed_watchdog()
                    continue
                await loop.run_in_executor(None, handle_message, data)
            elif msg.type == web.WSMsgType.ERROR:
                break
    finally:
        robot.stop()
        print("Controller disconnected - robot stopped.")
    return ws

def on_button(button, pressed):
    if button in ("forward", "backward", "left", "right"):
        if not pressed:
            robot.halt()
        elif button == "forward":
            if robot.is_trot_active():
                robot.forward()
            else:
                robot.height_up()
        elif button == "backward":
            if robot.is_trot_active():
                robot.backward()
            else:
                robot.height_down()
        elif button == "left":
            robot.left()
        elif button == "right":
            robot.right()
    elif button == "LB" and pressed:
        robot.activate()
    elif button == "A" and pressed:
        robot.activate()
    elif button == "RB" and pressed:
        robot.toggle_trot()
    elif button == "B" and pressed:
        robot.stop()
    elif button == "C" and pressed:
        robot.hop()
    elif button == "takeover" and pressed:
        robot.take_control()
    else:
        print(f"Button {button} {'pressed' if pressed else 'released'}")

def on_joystick(stick, x, y):
    robot.set_stick(stick, x, y)
    print(f"Joystick -> {stick:5} X: {x:5} | Y: {y:5}")

def handle_message(data):
    kind = data.get("type")
    if kind == "button":
        b = data.get("id")
        if b:
            on_button(b, bool(data.get("pressed")))
    elif kind == "joystick":
        on_joystick(data.get("stick", "?"),
                    round(float(data.get("x", 0)), 2),
                    round(float(data.get("y", 0)), 2))

def main():
    app = web.Application()
    app.add_routes([
        web.get("/", index),
        web.get("/ws", websocket),
        web.get("/favicon.ico", favicon),
    ])
    threading.Thread(target=voice_loop, daemon=True).start()
    print(">>> Web joystick server started at http://YOUR_PI_IP:5000")
    print(">>> You can now open the controller in your browser")
    web.run_app(app, host="0.0.0.0", port=5000)

if __name__ == "__main__":
    print(">>> ROBOT: Forcing clean stop on startup...")
    robot.stop()
    time.sleep(0.4)
    print(">>> Starting web joystick server...")
    asyncio.run(main())
