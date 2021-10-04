from RPi import GPIO as io
from time import sleep
io.setmode(io.BOARD)
some_freq = 440
dc = 50
servo = 12
buzzer_pin = 32
led = 10

io.setup(servo, io.OUT)
io.setup(buzzer_pin, io.OUT)
io.setup(led, io.OUT)

sg90 = io.PWM(servo, 50)
sg90.start(7.5)
buzzer =io.PWM(buzzer_pin, some_freq)
buzzer.start(0)

try:
    while True:
        io.output(led, io.LOW)
        sg90.ChangeDutyCycle(7.5)
        buzzer.ChangeDutyCycle(dc)
        sleep(1)
        sg90.ChangeDutyCycle(12.5)
        io.output(led, io.HIGH)
        sleep(1)
        sg90.ChangeDutyCycle(2.5)
        buzzer.ChangeDutyCycle(0)
        sleep(1)
finally:
    sg90.stop()
    buzzer.stop()
    io.cleanup()