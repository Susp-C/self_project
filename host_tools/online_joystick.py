import os
import json
import time
import asyncio
import threading
from aiohttp import web

import voice_control as robot

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ====================== VOICE STATE MACHINE ======================
# 语音输入现在来自手机浏览器（Web Speech API），通过 WebSocket 以
# {"type": "voice", "text": "<keyword>"} 发来。Pi 不再读本地麦克风。
_voice_awake = False
_voice_trot = False

VOICE_KEYWORDS = (
    "hello", "forward", "backward", "left", "right",
    "higher", "lower", "stop", "quit",
)


def on_voice(word):
    """Handle a single recognized keyword coming from the phone."""
    global _voice_awake, _voice_trot
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
        # already awake, ignore / re-confirm
        return

    elif word in ("higher", "lower"):
        if word == "higher":
            robot.height_up()
        else:
            robot.height_down()
        time.sleep(0.5)   # one step of height change
        robot.halt()      # hold at new height

    elif word in ("forward", "backward", "left", "right"):
        if not _voice_trot:
            robot.toggle_trot()   # enter trot only when movement needed
            _voice_trot = True
            time.sleep(0.3)
        getattr(robot, word)()    # robot.forward() / backward() / left() / right()

    elif word == "stop":
        if _voice_trot:
            robot.toggle_trot()   # exit trot first
            _voice_trot = False
        robot.stop()

    elif word == "quit":
        if _voice_trot:
            robot.toggle_trot()
            _voice_trot = False
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
        robot.stop()
        print("Controller disconnected - robot stopped.")
    return ws


def main():
    app = web.Application()
    app.add_routes([
        web.get("/", index),
        web.get("/ws", websocket),
        web.get("/favicon.ico", favicon),
    ])
    print(">>> Web joystick server started at http://YOUR_PI_IP:5000")
    print(">>> Open the controller in your phone browser")
    web.run_app(app, host="0.0.0.0", port=5000)


if __name__ == "__main__":
    print(">>> ROBOT: Forcing clean stop on startup...")
    robot.stop()
    time.sleep(0.4)
    print(">>> Starting web joystick server...")
    asyncio.run(main())
