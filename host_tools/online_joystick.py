import os
import json
import time
import asyncio
import socket
import ssl
import threading
from aiohttp import web

import voice_control as robot

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ====================== VOICE STATE MACHINE ======================
# Voice input comes from the phone browser (Web Speech API),
# sent over WebSocket as {"type": "voice", "text": "<keyword>"}.
_voice_awake = False

VOICE_KEYWORDS = (
    "hello", "forward", "backward", "left", "right",
    "higher", "lower", "stop", "quit",
)


def on_voice(word):
    """Handle a single recognized keyword coming from the phone."""
    global _voice_awake
    word = (word or "").strip().lower()
    if not word:
        return
    print(f"[voice] heard: {word}")

    # voice commands also keep the deadman watchdog alive
    robot.feed_watchdog()

    # --- not awake yet: only 'hello' wakes the robot ---
    if not _voice_awake:
        if word == "hello":
            _voice_awake = True
            print("[voice] awake — listening for movement commands")
        return

    # --- awake: handle commands ---
    if word == "hello":
        return

    elif word in ("higher", "lower"):
        if word == "higher":
            robot.height_up()
        else:
            robot.height_down()
        time.sleep(0.5)   # one step of height change
        robot.halt()      # hold at new height

    elif word in ("forward", "backward", "left", "right"):
        # FIX: use robot.is_trot_active() as ground truth instead of a local
        # flag — prevents desync when the watchdog or a reconnect clears trot
        # without this module knowing about it.
        if not robot.is_trot_active():
            robot.toggle_trot()
            time.sleep(0.3)
        getattr(robot, word)()    # robot.forward() / backward() / left() / right()

    elif word == "stop":
        if robot.is_trot_active():
            robot.toggle_trot()
        robot.stop()

    elif word == "quit":
        if robot.is_trot_active():
            robot.toggle_trot()
        robot.stop()
        _voice_awake = False
        print("[voice] sleeping. Say 'hello' to reactivate.")


# ====================== BUTTON / JOYSTICK ======================
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
    elif button == "ai_on" and pressed:
        robot.ai_on()
    elif button == "ai_off" and pressed:
        robot.ai_off()
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
    elif kind == "voice":
        on_voice(data.get("text", ""))


# ====================== WEB SERVER ======================
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
        # FIX: halt() zeroes motion axes without clearing trot state, so a
        # phone reconnect (or brief network hiccup) doesn't kill a voice-
        # initiated trot that is still supposed to be running.
        robot.halt()
        print("Controller disconnected - robot stopped.")
    return ws


def main():
    app = web.Application()
    app.add_routes([
        web.get("/", index),
        web.get("/ws", websocket),
        web.get("/favicon.ico", favicon),
    ])
    # Use HTTPS if cert/key exist alongside this script (required for Web Speech API
    # on non-localhost origins — browsers block microphone on plain HTTP)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cert = os.path.join(script_dir, 'cert.pem')
    key  = os.path.join(script_dir, 'key.pem')
    if os.path.exists(cert) and os.path.exists(key):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(cert, key)
        print(">>> Web joystick server started at https://YOUR_PI_IP:5000")
    else:
        ssl_ctx = None
        print(">>> Web joystick server started at http://YOUR_PI_IP:5000")
        print(">>> (no cert.pem/key.pem found — voice button needs Chrome flag workaround)")
    print(">>> Open the controller in your phone browser")
    # Pre-create socket with SO_REUSEADDR so restarts never hit "address in use"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', 5000))
    web.run_app(app, sock=sock, ssl_context=ssl_ctx)


if __name__ == "__main__":
    import sys
    print(f"[startup] PID={os.getpid()} running {__file__} via {sys.executable}")
    print(">>> ROBOT: Forcing clean stop on startup...")
    robot.stop()
    time.sleep(0.4)
    print(">>> Starting web joystick server...")
    asyncio.run(main())
