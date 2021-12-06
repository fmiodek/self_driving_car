#!/usr/bin/env python3

import time
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import LightSensor


def main():
    lsl = LightSensor(INPUT_1)
    lsr = LightSensor(INPUT_2)
    lsm = LightSensor(INPUT_3)
    while True:
        print(lsl.reflected_light_intensity, "links")
        print(lsr.reflected_light_intensity, "rechts")
        print(lsm.reflected_light_intensity, "mitte")
        time.sleep(5)


if __name__ == '__main__':
    main()
