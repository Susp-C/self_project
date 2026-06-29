#!/usr/bin/env python3
"""
listen.py — Offline voice control for Dingo robot dog.
Say 'hello' to wake the robot, then give movement commands.
Shares the vioce_control movement layer with online_joystick.py.
"""
import os
import sys
import json
import time

import vosk
import pyaudio

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vioce_control as robot

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vosk-model-small-en-us-0.15")
KEYWORDS = '["hello", "forward", "backward", "left", "right", "higher", "lower", "stop", "quit", "[unk]"]'
RATE = 48000
CHUNK = 8000
MIC_NAME = "AIMIC"        # USB combo mic+speaker; matched by name so card-number
                          # reordering (e.g. after replug) never breaks it
SPEAKER_DEVICE = "plughw:CARD=AIMICM4,DEV=0"


def speak(text):
    print(f"[ROBOT]: {text}")
    os.system(f'espeak "{text}" --stdout | aplay -D {SPEAKER_DEVICE} -q 2>/dev/null &')


def find_mic_index(pa):
    """Return the PyAudio input device index whose name contains MIC_NAME,
    so a card-number change never points us at the wrong device."""
    for i in range(pa.get_device_count()):
        d = pa.get_device_info_by_index(i)
        if d['maxInputChannels'] > 0 and MIC_NAME.lower() in d['name'].lower():
            print(f"[mic] using device {i}: {d['name']}")
            return i
    print(f"[mic] WARNING: '{MIC_NAME}' not found, falling back to default input")
    return None


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"ERROR: Model not found at {MODEL_PATH}")
        sys.exit(1)

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
        frames_per_buffer=CHUNK
    )
    stream.start_stream()

    print("Voice control ready. Say 'hello' to wake the robot.")
    awake = False
    trot_active = False

    try:
        while True:
            data = stream.read(CHUNK, exception_on_overflow=False)
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                word = result.get("text", "").strip()
                if not word or word == "[unk]":
                    continue

                print(f"[Heard]: {word}")

                if not awake:
                    if word == "hello":
                        awake = True
                        robot.activate()   # L1 only — no trot yet, dog just stands
                        speak("ready")
                else:
                    if word in ("higher", "lower"):
                        speak(word)
                        if word == "higher":
                            robot.height_up()
                        else:
                            robot.height_down()
                        time.sleep(0.5)   # hold rate for 0.5 s → one step of height change
                        robot.halt()      # stop rate, robot holds new height
                    elif word in ("forward", "backward", "left", "right"):
                        if not trot_active:
                            robot.toggle_trot()   # enter trot only when movement needed
                            trot_active = True
                            time.sleep(0.3)
                        if word == "forward":
                            speak("forward")
                            robot.forward()
                        elif word == "backward":
                            speak("backward")
                            robot.backward()
                        elif word == "left":
                            speak("left")
                            robot.left()
                        elif word == "right":
                            speak("right")
                            robot.right()
                    elif word == "stop":
                        speak("stop")
                        if trot_active:
                            robot.toggle_trot()   # exit trot first (flag still True here)
                            trot_active = False
                        robot.stop()
                    elif word == "quit":
                        speak("goodbye")
                        if trot_active:
                            robot.toggle_trot()   # exit trot first (flag still True here)
                            trot_active = False
                        robot.stop()
                        awake = False
                        print("Sleeping. Say 'hello' to reactivate.")

    except KeyboardInterrupt:
        print("\nShutting down.")
        robot.stop()
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()


if __name__ == "__main__":
    main()
