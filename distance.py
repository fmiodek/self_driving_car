#!/usr/bin/env python3

import time
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor


def main():
    us = UltrasonicSensor(INPUT_4)
    while True:
        print(us.distance_centimeters)
        time.sleep(5)


if __name__ == '__main__':
    main()
