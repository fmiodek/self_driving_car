#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import ev3dev2
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import LightSensor, UltrasonicSensor


class StateMachine(object):

    def __init__(self):
        self.BLACK_SPLIT = 40  # Threshold for black color reflection; \
        # everything below is (most probably) black
        self.WHITE_SPLIT = 40  # Threshold for white color reflection; \
        # everything above is (most probably) white
        self.FULL_SPEED = 30 * 1.25  # speed for going straight
        self.CURVE_SPEED = 20 * 1.25  # speed for inner wheel in a normal curve
        self.SPEED_90 = 20 * 1.25  # speed for outer wheel in a sharp curve
        self.BACK_SPEED = -20 * 1.25  # speed for inner wheel in a sharp curve
        self.THRESHOLD = 10  # Difference-threshold between black and white reflection
        self.STATE_GO_ON = 1  # state when driving
        self.SATE_WAIT = 2  # state when waiting at an obstacle
        self.STATE_STOP = 3  # state after reaching the finish line
        self.BLACK_COUNT = 0  # Counter when all sensors detect black, \
        # to check if white is following in a given time; relevant for finish detection

        self.mt = MoveTank(OUTPUT_A, OUTPUT_D)
        self.lsl = LightSensor(INPUT_1)
        self.lsr = LightSensor(INPUT_3)
        self.lsm = LightSensor(INPUT_2)
        self.us = UltrasonicSensor(INPUT_4)
        self.state = self.STATE_GO_ON  # initialize with state for driving

    # Calculates difference in light detection between left and right sensor
    def diff_left_right(self):
        return self.lsl.reflected_light_intensity - self.lsr.reflected_light_intensity

    # Calculates difference in light detection between left and middle sensor
    def diff_left_mid(self):
        return self.lsl.reflected_light_intensity - self.lsm.reflected_light_intensity

    # Calculates difference in light detection between right and middle sensor
    def diff_right_mid(self):
        return self.lsr.reflected_light_intensity - self.lsm.reflected_light_intensity

    # if left and front sensor are on black and right sensor on white, it´s time to turn left
    def time_to_go_left(self):
        if self.diff_left_right()*(-1) > self.THRESHOLD and self.diff_left_mid()*(-1) < self.THRESHOLD:
            return True

    # if right and front sensor are on black and left sensor on white, it´s time to turn left
    def time_to_go_right(self):
        if self.diff_left_right() > self.THRESHOLD and self.diff_right_mid()*(-1) < self.THRESHOLD:
            return True

    # if left sensor is on black and both right and mid sensor on white, it´s time to turn left sharp
    def time_to_turn_left_90(self):
        if self.diff_left_right()*(-1) > self.THRESHOLD and self.diff_left_mid()*(-1) > self.THRESHOLD:
            return True

    # if right sensor is on black and both left and mid sensor on white, it´s time to turn right sharp
    def time_to_turn_right_90(self):
        if self.diff_left_right() > self.THRESHOLD and self.diff_right_mid()*(-1) > self.THRESHOLD:
            return True

    # if middle sensor is on black and both left and right sensor on white, it´s time to go straight
    def time_to_go_on(self):
        if self.diff_left_mid() > self.THRESHOLD and self.diff_right_mid() > self.THRESHOLD:
            return True

    # this function detects when all sensors detect the same color, e.g. at finish line
    def all_equal(self):
        if self.diff_left_right() < self.THRESHOLD and self.diff_left_right()*(-1) < self.THRESHOLD and \
                self.diff_left_mid() < self.THRESHOLD and self.diff_left_mid()*(-1) < self.THRESHOLD and \
                self.diff_right_mid() < self.THRESHOLD and self.diff_right_mid()*(-1) < self.THRESHOLD:
            return True

    # this function detects when all sensors detect the black, e.g. at finish line
    def all_black(self):
        if self.lsm.reflected_light_intensity < self.BLACK_SPLIT and self.lsl.reflected_light_intensity < self.BLACK_SPLIT and self.lsr.reflected_light_intensity < self.BLACK_SPLIT:
            return True

    # this function detects when all sensors detect white color, e.g. behind finish line or line interruption
    def all_white(self):
        if self.lsm.reflected_light_intensity > self.WHITE_SPLIT and self.lsl.reflected_light_intensity > self.WHITE_SPLIT and self.lsr.reflected_light_intensity > self.WHITE_SPLIT:
            return True

    # this function checks if all sensors are on black. if so, it activates a counter
    def check_black_3(self):
        if self.all_black():
            self.BLACK_COUNT = 6

    # this function checks if all sensors are on white. If in addition the Counter is above zero, \
    # the robot detects a finish line.
    def time_to_stop(self):
        if self.all_white() and self.BLACK_COUNT > 0:
            return True

    # if Ultrasonic sensor detects an obstacle, it´s time to wait
    def time_to_wait(self):
        if self.us.distance_centimeters < 7:
            return True

    # sets motorspeeds for normal left curve
    def turn_left(self):
        self.mt.on(SpeedPercent(self.CURVE_SPEED),
                   SpeedPercent(self.FULL_SPEED))

    # sets motorspeeds for normal right curve
    def turn_right(self):
        self.mt.on(SpeedPercent(self.FULL_SPEED),
                   SpeedPercent(self.CURVE_SPEED))

    # sets motorspeeds for sharp left curve
    def turn_90_left(self):
        self.mt.on(SpeedPercent(self.BACK_SPEED), SpeedPercent(self.SPEED_90))

    # sets motorspeeds for sharp right curve
    def turn_90_right(self):
        self.mt.on(SpeedPercent(self.SPEED_90), SpeedPercent(self.BACK_SPEED))

    # sets motorpseeds for going straight
    def go_straight(self):
        self.mt.on(SpeedPercent(self.FULL_SPEED),
                   SpeedPercent(self.FULL_SPEED))

    # this function sets the conditions when the robot should do his actions
    def drive_on(self):
        if self.time_to_wait():
            self.mt.off()
            self.state = self.SATE_WAIT
            return
        if self.time_to_stop():
            self.mt.off()
            self.state = self.STATE_STOP
            return
        if self.time_to_go_on():
            self.go_straight()
        if self.time_to_go_left():
            self.turn_left()
        if self.time_to_go_right():
            self.turn_right()
        if self.time_to_turn_left_90():
            self.turn_90_left()
        if self.time_to_turn_right_90():
            self.turn_90_right()
        if self.all_white():
            self.go_straight()
        else:
            self.state = self.STATE_GO_ON

    def main(self):
        while True:
            # in every loop, check if all 3 sensors are on black. If yes, set the counter.
            self.check_black_3()
            # if the counter is above zero, reduce it with every loop,
            # this way the robot can differenciate between the FINISH LINE,
            # where 3 blacks are directly followed by 3 whites, so the counter is above zero
            # and LINE INTERRUPTIONS, where are not 3 blacks directly before 3 whites,
            # so the counter is on zero when detecting 3 whites.
            if self.BLACK_COUNT > 0:
                self.BLACK_COUNT -= 1
            if self.state == self.STATE_GO_ON:
                self.drive_on()
            if self.state == self.SATE_WAIT:
                # wait until the obstacle is removed
                if self.time_to_wait():
                    self.state = self.SATE_WAIT
                else:
                    self.state = self.STATE_GO_ON
            if self.state == self.STATE_STOP:
                print(self.BLACK_COUNT)
                time.sleep(5)
                break
            time.sleep(0.01)


if __name__ == '__main__':
    sm = StateMachine()
    sm.main()
