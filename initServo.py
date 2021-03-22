#!/usr/bin/python3

import time
import PCA9685

pwm = PCA9685.PCA9685(address=0x60)
pwm.setPWMFreq(50)

while 1:
    for i in range(0,16):
        pwm.setPWM(i, 0, 300)
    time.sleep(3)