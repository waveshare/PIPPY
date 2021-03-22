#!/usr/bin/env/python3
# File name   : robot.py
# Description : Ctrl Robot
import OLED
import PIPPY
import time
import Voltage

screen = OLED.OLED_ctrl()
screen.start()

# OLED screen ctrl
# screen.screen_show(position, text)
# screen.screen_show(1, 'IP:192.168.12.1')

robotCtrl = PIPPY.PIPPY()
robotCtrl.start()

pitch, roll = 0, 0


def forward(speed=100):
	robotCtrl.moveStart(speed, 'forward', 'no')


def backward(speed=100):
	robotCtrl.moveStart(speed, 'backward', 'no')


def left(speed=100):
	robotCtrl.moveStart(speed, 'no', 'left')


def right(speed=100):
	robotCtrl.moveStart(speed, 'no', 'right')


def stop():
	lookForward()
	robotCtrl.moveStop()


def steadyModeStart():
	robotCtrl.functionSelect('steady')


def lookForward():
	global pitch, roll
	pitch = 0
	roll = 0


def lookUp(speed=7):
	global pitch
	pitchBuffer = pitch + speed
	if pitchBuffer < (PIPPY.maxHeight - PIPPY.middleHeight) and pitchBuffer > -(PIPPY.maxHeight - PIPPY.middleHeight):
		pitch = pitchBuffer
	
	try:
		PIPPY.pitchRoll(pitch, roll)
	except:
		pass


def lookDown(speed=7):
	global pitch
	pitchBuffer = pitch - speed
	if pitchBuffer < (PIPPY.maxHeight - PIPPY.middleHeight) and pitchBuffer > -(PIPPY.maxHeight - PIPPY.middleHeight):
		pitch = pitchBuffer

	try:
		PIPPY.pitchRoll(pitch, roll)
	except:
		pass


def leanLeft(speed=7):
	global roll
	roll += speed
	PIPPY.pitchRoll(pitch, roll)


def leanRight(speed=7):
	global roll
	roll -= speed
	PIPPY.pitchRoll(pitch, roll)


def stayLow():
	PIPPY.stay(PIPPY.minHeight)


def standUp():
	PIPPY.stay(PIPPY.maxHeight)


def getVoltage():
	return round(Voltage.measure(), 2)


def getUltrasonic():
	return None


if __name__ == '__main__':
    # robotCtrl.moveStart(100, 'forward', 'no', 0)
    # time.sleep(3)
    # robotCtrl.moveStop()
    while 1:
    	print(getVoltage())
    	time.sleep(1)
    	pass