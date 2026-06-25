import os
import json
import asyncio
from aiohttp import web
import voice_control as robot

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

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
                    robot.feed_watchdog()       # heartbeats keep the deadman alive
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
            robot.stop()
        elif button == "forward":
            robot.forward()
        elif button == "backward":
            robot.backward()
        elif button == "left":
            robot.left()
        elif button == "right":
            robot.right()
    elif button == "A" and pressed:
        robot.activate()         # tap A once to enable walking (L1 + R1)
    elif button == "RB" and pressed:
        robot.toggle_trot()      # RB toggles trot <-> stand (R1 only)
    elif button == "B" and pressed:
        robot.stop()             # B = emergency STOP (zero all motion)
    elif button == "takeover" and pressed:
        robot.take_control()
    else:
        print(f"Button {button} {'pressed' if pressed else 'released'}")

def on_joystick(stick, x, y):
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
    web.run_app(app, host="0.0.0.0", port=5000)

if __name__ == "__main__":
    main()
