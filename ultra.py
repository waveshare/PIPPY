#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

Tr = 12
Ec = 13

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(Ec, GPIO.IN)

disListNum = 3

def checkdist_bk():       #Reading distance
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(Tr, GPIO.LOW)

    while not GPIO.input(Ec):
        pass
    t1 = time.time()
    while GPIO.input(Ec):
        pass
    t2 = time.time()

    distanceGet = round((t2-t1)*340/2,2)

    if distanceGet > 3:
        distanceGet = 3

    return distanceGet


def checkdist_base():       #Reading distance
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(Tr, GPIO.LOW)

    t1 = time.time()
    while not GPIO.input(Ec):
        t1 = time.time()

    while GPIO.input(Ec):
        t2 = time.time()

    distanceGet = round((t2-t1)*340/2,2)

    if distanceGet > 3:
        distanceGet = 3

    return distanceGet


def checkdist():
    disList = []
    for i in range(0, disListNum):
        disList.append(checkdist_base())

    return min(disList)

if __name__ == '__main__':
    while 1:
        print(checkdist())
        time.sleep(1)
