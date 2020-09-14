#!/usr/bin/env micropython

from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

ts = TouchSensor(INPUT_1)
leds = Leds()

print("Press the touch sensor to change the LED color!")

# m = LargeMotor(OUTPUT_A)
# m.on_for_rotations(SpeedPercent(75), 5)

from ev3dev2.sound import Sound

sound = Sound()
sound.speak("LICKo is going to win it all this year!  Lets goooooooooooo!")

flip = True
while True:
    # if ts.is_pressed:
    if flip:
        leds.set_color("LEFT", "GREEN")
        leds.set_color("RIGHT", "GREEN")
    else:
        leds.set_color("LEFT", "RED")
        leds.set_color("RIGHT", "RED")
    flip = not flip
    # don't let this loop use 100% CPU
    sleep(0.2)
    