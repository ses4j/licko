#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (
    ColorSensor,
    GyroSensor,
    InfraredSensor,
    Motor,
    TouchSensor,
    UltrasonicSensor,
)
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import ImageFile, SoundFile
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
medium_motor = Motor(Port.D)


robot = DriveBase(left_motor, right_motor, wheel_diameter=95, axle_track=94)

# For robot.settings, the four numbers mean straight speed, straight acelleration, turn rate, turn acceleration.
robot.settings(800, 200, 100, 50)

ev3 = EV3Brick()


def go_straight_then_stop(distance, speed):
    (
        straight_speed,
        straight_acceleration,
        turn_rate,
        turn_acceleration,
    ) = robot.settings()
    robot.stop()
    robot.settings(speed, straight_acceleration, turn_rate, turn_acceleration)
    robot.straight(distance)
    robot.stop()


def StepCounter_M02():
    pass
    # go forward
    (
        straight_speed,
        straight_acceleration,
        turn_rate,
        turn_acceleration,
    ) = robot.settings()
    print(straight_speed, straight_acceleration, turn_rate, turn_acceleration)

    go_straight_then_stop(-1500, 400)
    go_straight_then_stop(30, 150)
    go_straight_then_stop(-325, 50)
    go_straight_then_stop(1500, 450)


def WeightMachine_M13():
    # Initialize the motors.
    # ev3.speaker.say("LICKo is the best and will definitely win!")

    # go forward
    medium_motor.run(-100)
    print("about to straight")
    robot.straight(distance=25000)

    # drop arm
    # print("about to runtime")
    # medium_motor.run_time(speed=1000, time=1000, then=Stop.HOLD, wait=True)
    # print("about to runstall")
    # medium_motor.run_until_stalled(-100)
    # robot.straight(distance=-250)

    # Initialize the motors.

    # ev3 = EV3Brick()
    # ev3.speaker.say("LICKo is the best and will definitely win!")

    # go forward
    robot.straight(distance=1000)
    # left_motor.run(50)
    # right_motor.run(50)

    # drop arm
    # medium_motor.run_time(speed=1000, time=1000, then=Stop.HOLD, wait=True)
    def move_arm_up_hard():
        medium_motor.run_time(speed=-100, time=2000, then=Stop.HOLD, wait=True)

    def move_arm_up():
        medium_motor.run_until_stalled(-100, then=Stop.HOLD)

    def move_arm_down():
        medium_motor.run_until_stalled(100, then=Stop.HOLD)

    robot.straight(distance=-100)
    move_arm_down()
    move_arm_up()

    robot.straight(distance=-200)
    move_arm_down()
    robot.straight(distance=300)
    move_arm_up_hard()


def WeightMachine_M13_from_steves_computer():
    # Initialize the motors.

    # ev3.speaker.say("LICKo is the best and will definitely win!")

    # go forward
    robot.straight(distance=250)

    # drop arm
    medium_motor.run_time(speed=1000, time=1000, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(-100)

    # robot.straight(-250)

    # go back 250mm
    robot.straight(distance=-250)
    # medium_motor.run(10)
    # wait(10000)


def Basket_05():
    # robot.settings(800,200,100,50)
    ev3.speaker.say("LICKo is the best")
    robot.straight(distance=600)
    robot.turn(-200)
    robot.straight(distance=1111)
    robot.turn(-100)
    robot.straight(distance=128)
    # ev3.speaker.say("Robot drops green thing into basket")
    medium_motor.run_time(speed=550, time=400, then=Stop.HOLD, wait=True)
    medium_motor.run_time(speed=-250, time=400, then=Stop.HOLD, wait=True)
    medium_motor.run_time(speed=550, time=400, then=Stop.HOLD, wait=True)
    # arm goes up
    # ev3.speaker.say("arm goes up")
    medium_motor.run_until_stalled(-500)
    # robot goes back
    # ev3.speaker.say("robot goes back")
    robot = DriveBase(left_motor, right_motor, wheel_diameter=95, axle_track=94)
    robot.straight(distance=-200)
    # arm goes to the ground until it can't go farther
    # ev3.speaker.say("arm goes to the ground until it can't")
    medium_motor.run_time(speed=1000, time=125, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(1000)
    # arm goes up
    # ev3.speaker.say("arm goes up")
    medium_motor.run_angle(speed=50, rotation_angle=-55, then=Stop.HOLD, wait=False)
    # ev3.speaker.say("Gooooooo!")
    robot.straight(distance=325)
    # medium_motor.run_time(speed=-1700, time=400, then=Stop.HOLD, wait=True)
    medium_motor.run_angle(speed=5000, rotation_angle=-250, then=Stop.HOLD, wait=True)
    wait(10000)


def TestRuns():
    robot = DriveBase(left_motor, right_motor, wheel_diameter=46, axle_track=102)

    ev3 = EV3Brick()
    robot.settings(800, 200, 100, 50)
    GoBackwards(bot=robot, distance=-70)
    # robot.turn(angle=360)
    # robot.straight(distance=30)
    # robot.turn(angle=360)
    # robot.straight(distance=1220)
    # medium_motor.run_time(speed=500, time=325, then=Stop.HOLD, wait=True)
    # medium_motor.run_time(speed=-300, time=425, then=Stop.HOLD, wait=True)
    # medium_motor.run_time(speed=300, time=425, then=Stop.HOLD, wait=True)
    # medium_motor.run_time(speed=-300, time=425, then=Stop.HOLD, wait=True)
    # medium_motor.run_time(speed=300, time=325, then=Stop.HOLD, wait=True)
    # medium_motor.run_until_stalled(1000)


def LineFollow(distance):
    # Initialize the color sensor.
    line_sensor = ColorSensor(Port.S3)
    # Calculate the light threshold. Choose values based on your measurements.
    BLACK = 0
    WHITE = 100
    threshold = (BLACK + WHITE) / 2

    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 201

    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # For example, if the light value deviates from the threshold by 10, the robot
    # steers at 10*1.2 = 12 degrees per second.
    PROPORTIONAL_GAIN = 1.2

    # Start following the line endlessly.
    go_yes_or_no = True
    while go_yes_or_no:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        print(robot.distance())
        if robot.distance() >= distance:
            go_yes_or_no = False
            robot.stop()
            robot.reset()
        # You can wait for a short time or do other things in this loop.
        wait(10)


def Bench_M04():
    # Initialize the motors.

    robot = DriveBase(left_motor, right_motor, wheel_diameter=95, axle_track=94)

    robot.settings(800, 200, 100, 50)
    ev3 = EV3Brick()
    # ev3.speaker.say("LICKo is the best and will definitely win!")

    # go forward
    robot.straight(distance=720)
    # robot.turn(angle=-10)
    robot.turn(angle=30)
    # robot.straight(distance=-100)
    medium_motor.run_time(speed=750, time=125, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(1000)
    robot.straight(distance=-250)
    # medium_motor.run_time(speed=2500, time=500, then=Stop.HOLD, wait=True)
    # medium_motor.run_time(speed=-100, time=75, then=Stop.HOLD, wait=True)
    robot.straight(distance=425)
    medium_motor.run_time(speed=-1000, time=925, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(1000)
    ev3.speaker.say("YAY!")
    ev3.speaker.say("Sigh...")


def GoBackwards(bot, distance):
    bot.straight(distance)


def Treadmill():
    medium_motor_2 = Motor(Port.A)
    medium_motor.run_time(speed=1450, time=175, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(2000)
    medium_motor_2.run_time(speed=1500, time=7000, then=Stop.HOLD, wait=True)
    # medium_motor_2.run_time(speed=200, time=3000, then=Stop.HOLD, wait=True)
    # Reminder to line follow up to treadmill and back


def testdrop():
    medium_motor_2 = Motor(Port.A)
    # medium_motor.run_time(speed=1450, time=275, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(1450, then=Stop.HOLD)
    # medium_motor.run_time(speed=-1450, time=875, then=Stop.HOLD, wait=True)
    # medium_motor.run_angle(speed=-1500, rotation_angle=-60, then=Stop.HOLD, wait=True)
    medium_motor.run_until_stalled(-4000, then=Stop.HOLD)


def testspin():
    medium_motor_2 = Motor(Port.A)
    medium_motor_2.run_time(speed=1500, time=7000, then=Stop.HOLD, wait=True)


def Turnbot(angle):
    robot.turn(angle=angle)
    robot.reset()


def CheckColors():
    line_sensor = ColorSensor(Port.S3)
    while True:
        print(line_sensor.reflection())
        wait(1)


def M11_Treadmill():
    LineFollow(2525)
    robot.straight(distance=65)
    Treadmill()
    robot.straight(distance=-3000)


def go_under_pullup_bar_M6():
    robot.straight(distance=1700)
    robot.turn(angle=-200)
    robot.straight(distance=1100)
    robot.turn(angle=-200)
    robot.straight(distance=900)
    robot.turn(angle=-125)
    robot.straight(distance=1600)
    # robot.turn(angle=200)
    # robot.straight(distance=500)


# Basket_05()

# Turnbot(400)

# TestRuns()

# M11_Treadmill()

testdrop()

# go_under_pullup_bar_M6()

# testspin()

# LineFollow(1000)

# CheckColors()

# medium_motor = Motor(Port.C)
# medium_motor.run_time(speed=-1000, time=400, then=Stop.HOLD, wait=True)


# To Do: Finish basket, treadmill, bench, and step counter
# Re-build basket arm (Make it wider by using the red pegs and green arms)

# StepCounter_M02()
# Bench_M04()
# WeightMachine_M13()
# Basketball_M05()
