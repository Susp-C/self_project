import RPi.GPIO as GPIO
import time

dc_pin = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(dc_pin, GPIO.OUT)

# 让它以明显的频率跳变，看看示波器有没有反应
while True:
    GPIO.output(dc_pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(dc_pin, GPIO.LOW)
    time.sleep(0.5)
