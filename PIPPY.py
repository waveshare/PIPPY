#!/usr/bin/python3

import PCA9685
import numpy as np 
import time
import threading
import RPi.GPIO as GPIO
import os
import ICM20948
import Kalman_filter

pwm = PCA9685.PCA9685(address=0x60)
pwm.setPWMFreq(50)

imu = ICM20948.ICM20948()
kfX = Kalman_filter.Kalman_filter(0.01,0.1)
kfY = Kalman_filter.Kalman_filter(0.01,0.1)

'''
0 autoSelect
1 diagonalSelect
2 triangularSelect
'''
selectGait = 0
speedApart = 50


'''
4---1 
5---0


6---3
7---2
'''

init_pwm0 = 300
init_pwm1 = 300
init_pwm2 = 300
init_pwm3 = 300

init_pwm4 = 300
init_pwm5 = 300
init_pwm6 = 300
init_pwm7 = 300

init_pwm8 = 300
init_pwm9 = 300
init_pwm10 = 300
init_pwm11 = 300

init_pwm12 = 300
init_pwm13 = 300
init_pwm14 = 300
init_pwm15 = 300


buffer_8  = []
buffer_9  = []
buffer_10 = []
buffer_11 = []

buffer_8A = 0
buffer_8B = 0

buffer_9A = 0
buffer_9B = 0

buffer_10A = 0
buffer_10B = 0

buffer_11A = 0
buffer_11B = 0

initPos = [init_pwm0,init_pwm1,init_pwm2,init_pwm3,
           init_pwm4,init_pwm5,init_pwm6,init_pwm7,
           init_pwm8,init_pwm9,init_pwm10,init_pwm11,
           init_pwm12,init_pwm13,init_pwm14,init_pwm15]

lastPos = [0,50, 0,50, 0,50, 0,50]
goalPos = [0,50, 0,50, 0,50, 0,50]
nowPos = [0,50, 0,50, 0,50, 0,50]

sc_direction = [-1,-1,-1,-1, 1,1,1,1, 1,1,1,1, 1,1,1,1]
DPI = 1
delayTime = 0.001

ctrlRangeMax = 500
ctrlRangeMin = 100
angleRange = 180

walkWiggle = 25.0
walkHeight = 70.0
liftHeight = 6.0

walkOffset = 5.0
middleOffset = 0.0

maxHeight = 85.0
minHeight = 60.0
middleHeight = (maxHeight + minHeight)/2

offSetD = 0.0

distanceCheak = 0.6

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

linkageLenA = 23
linkageLenB = 36.9127
linkageLenC = 19.6543
linkageLenD = 32.5225

linkageLenE = 12.5

servoNumCtrl = [0,1]
servoDirection = [1,-1]

sinput = 1


def limitCheck(posInput, circlePos, circleLen, outline): #E
    circleRx = posInput[0]-circlePos[0]
    circleRy = posInput[1]-circlePos[1]
    realPosSquare = circleRx*circleRx+circleRy*circleRy
    shortRadiusSquare = np.square(circleLen[1]-circleLen[0])
    longRadiusSquare = np.square(circleLen[1]+circleLen[0])

    if realPosSquare >= shortRadiusSquare and realPosSquare <= longRadiusSquare:
        return posInput[0], posInput[1]

    else:
        lineK = (posInput[1]-circlePos[1])/(posInput[0]-circlePos[0])
        lineB = circlePos[1]-(lineK*circlePos[0])
        
        if realPosSquare < shortRadiusSquare:
            aX = 1 + lineK*lineK
            bX = 2*lineK*(lineB - circlePos[1]) - 2*circlePos[0]
            cX = circlePos[0]*circlePos[0] + (lineB - circlePos[1])*(lineB - circlePos[1]) - shortRadiusSquare

            resultX = bX*bX - 4*aX*cX
            x1 = (-bX + np.sqrt(resultX))/(2*aX)
            x2 = (-bX - np.sqrt(resultX))/(2*aX)

            y1 = lineK*x1 + lineB
            y2 = lineK*x2 + lineB

            if posInput[0] > circlePos[0]:
                if x1 > circlePos[0]:
                    xGenOut = x1+outline
                    yGenOut = y1
                else:
                    xGenOut = x2-outline
                    yGenOut = y2
            elif posInput[0] < circlePos[0]:
                if x1 < circlePos[0]:
                    xGenOut = x1-outline
                    yGenOut = y1
                else:
                    xGenOut = x2+outline
                    yGenOut = y2
            elif posInput[0] == circlePos[0]:
                if posInput[1] > circlePos[1]:
                    if y1 > circlePos[1]:
                        xGenOut = x1
                        yGenOut = y1+outline
                    else:
                        xGenOut = x2
                        yGenOut = y2-outline

            return xGenOut, yGenOut

        elif realPosSquare > longRadiusSquare:
            aX = 1 + lineK*lineK
            bX = 2*lineK*(lineB - circlePos[1]) - 2*circlePos[0]
            cX = circlePos[0]*circlePos[0] + (lineB - circlePos[1])*(lineB - circlePos[1]) - longRadiusSquare

            resultX = bX*bX - 4*aX*cX
            x1 = (-bX + np.sqrt(resultX))/(2*aX)
            x2 = (-bX - np.sqrt(resultX))/(2*aX)

            y1 = lineK*x1 + lineB
            y2 = lineK*x2 + lineB

            if posInput[0] > circlePos[0]:
                if x1 > circlePos[0]:
                    xGenOut = x1-outline
                    yGenOut = y1
                else:
                    xGenOut = x2+outline
                    yGenOut = y2
            elif posInput[0] < circlePos[0]:
                if x1 < circlePos[0]:
                    xGenOut = x1+outline
                    yGenOut = y1
                else:
                    xGenOut = x2-outline
                    yGenOut = y2
            elif posInput[0] == circlePos[0]:
                if posInput[1] > circlePos[1]:
                    if y1 > circlePos[1]:
                        xGenOut = x1
                        yGenOut = y1-outline
                    else:
                        xGenOut = x2
                        yGenOut = y2+outline

            return xGenOut, yGenOut


def planeLinkageReverse(linkageLen, linkageEnDe, servoNum, debugPos, goalPos): #E
    goalPos[0] = goalPos[0] + debugPos[0]
    goalPos[1] = goalPos[1] + debugPos[1]

    AngleEnD = np.arctan(linkageEnDe/linkageLen[1])*180/np.pi

    linkageLenREAL = np.sqrt(((linkageLen[1]*linkageLen[1])+(linkageEnDe*linkageEnDe)))

    goalPos[0],goalPos[1] = limitCheck(goalPos, debugPos, [linkageLen[0],linkageLenREAL], 0.00001)

    if goalPos[0] < 0:
        goalPos[0] = - goalPos[0]
        mGenOut = linkageLenREAL*linkageLenREAL-linkageLen[0]*linkageLen[0]-goalPos[0]*goalPos[0]-goalPos[1]*goalPos[1]
        nGenOut = mGenOut/(2*linkageLen[0])

        angleGenA = np.arctan(goalPos[1]/goalPos[0])+np.arcsin(nGenOut/np.sqrt(goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))
        angleGenB = np.arcsin((goalPos[1]-linkageLen[0]*np.cos(angleGenA))/linkageLenREAL)-angleGenA

        angleGenA = 90 - angleGenA*180/np.pi
        angleGenB = angleGenB*180/np.pi

        linkageLenC = np.sqrt((goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))

        linkagePointC = np.arcsin(goalPos[0]/goalPos[1])*180/np.pi*servoDirection[servoNumCtrl[0]]

        anglePosC = angleGenB + angleGenA

        return [angleGenA*servoDirection[servoNumCtrl[0]], (angleGenB+AngleEnD)*servoDirection[servoNumCtrl[1]], linkageLenC, linkagePointC, anglePosC]

    elif goalPos[0] == 0:
        angleGenA = np.arccos((linkageLen[0]*linkageLen[0]+goalPos[1]*goalPos[1]-linkageLenREAL*linkageLenREAL)/(2*linkageLen[0]*goalPos[1]))
        cGenOut = np.tan(angleGenA)*linkageLen[0]
        dGenOut = goalPos[1]-(linkageLen[0]/np.cos(angleGenA))
        angleGenB = np.arccos((cGenOut*cGenOut+linkageLenREAL*linkageLenREAL-dGenOut*dGenOut)/(2*cGenOut*linkageLenREAL))

        angleGenA = - angleGenA*180/np.pi + 90
        angleGenB = - angleGenB*180/np.pi

        linkageLenC = np.sqrt((goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))

        linkagePointC = angleGenB + 90 - angleGenA

        anglePosC = angleGenB + angleGenA

        return [angleGenA*servoDirection[servoNumCtrl[0]], (angleGenB+AngleEnD)*servoDirection[servoNumCtrl[1]], linkageLenC, linkagePointC, anglePosC]

    elif goalPos[0] > 0:
        sqrtGenOut = np.sqrt(goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1])
        nGenOut = (linkageLen[0]*linkageLen[0]+goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]-linkageLenREAL*linkageLenREAL)/(2*linkageLen[0]*sqrtGenOut)
        angleA = np.arccos(nGenOut)*180/np.pi

        AB = goalPos[1]/goalPos[0]

        angleB = np.arctan(AB)*180/np.pi
        angleGenA = angleB - angleA

        mGenOut = (linkageLen[0]*linkageLen[0]+linkageLenREAL*linkageLenREAL-goalPos[0]*goalPos[0]-goalPos[1]*goalPos[1])/(2*linkageLen[0]*linkageLenREAL)
        angleGenB = np.arccos(mGenOut)*180/np.pi - 90

        linkageLenC = np.sqrt((goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))

        # linkagePointC = np.arcsin(goalPos[1]/goalPos[0])*180/np.pi*servoDirection[servoNumCtrl[0]]
        linkagePointC = 0

        anglePosC = angleGenB + angleGenA

        return [angleGenA*servoDirection[servoNumCtrl[0]], (angleGenB+AngleEnD)*servoDirection[servoNumCtrl[1]], linkageLenC, linkagePointC, anglePosC]


def planeLinkageDouble(linkageLen, goalPos): #E
    goalPos[0],goalPos[1] = limitCheck(goalPos, [0,0], [linkageLen,linkageLen], 0.00001)

    if goalPos[0]<=0:
        # goalPos[0] = - goalPos[0]

        lineF = np.sqrt(goalPos[0]**2 + goalPos[1]**2)
        largeAng = np.arccos((lineF**2)/(2*linkageLen*lineF))*180/np.pi
        smallAng = np.arctan(goalPos[0]/goalPos[1])*180/np.pi

        angleGenA = largeAng - smallAng
        # print(smallAng)
        return angleGenA
    elif goalPos[0]>0:
        lineF = np.sqrt(goalPos[0]**2 + goalPos[1]**2)
        angL = np.arcsin(goalPos[0]/lineF)*180/np.pi
        angR = np.arccos((lineF**2)/(2*linkageLen*lineF))*180/np.pi
        angleGenA = angL + angR
        return angleGenA


def middlePosGenOut(linkageLen, angleInputA, angleInputB):
    angleInputA = -angleInputA
    angleV = 90 + angleInputB
    lineF  = np.sqrt(-np.cos(angleV*np.pi/180)*2*linkageLen[0]*linkageLen[1] + linkageLen[0]**2 + linkageLen[1]**2)
    angleO = np.arccos((linkageLen[0]**2 + lineF**2 - linkageLen[1]**2)/(2*linkageLen[0]*lineF))*180/np.pi
    
    if angleO < angleInputA:
        angleU = (angleInputA - angleO)*np.pi/180
        middleOutX = np.sin(angleU)*lineF
        middleOutY = np.cos(angleU)*lineF
        return [-middleOutX, middleOutY] 
        # print(-middleOutX)
    if angleO > angleInputB:
        angleU = (angleO - angleInputA)*np.pi/180
        middleOutX = np.sin(angleU)*lineF
        middleOutY = np.cos(angleU)*lineF
        return [middleOutX, -middleOutY] 


def animateLine(xInput, yInput, lineLen, angleInput, debugInput):
    angleOri = angleInput
    debugOri = debugInput

    aOut = angleInput + debugInput

    angleLine = aOut

    if angleLine < -360:
        angleLine = (-angleLine-360)*np.pi/180
        xOut = np.cos(angleLine)*lineLen + xInput
        yOut = -np.sin(angleLine)*lineLen + yInput

    elif angleLine < -180 and angleLine >= -360:
        angleLine = (-angleLine-180)*np.pi/180
        xOut = xInput - np.cos(angleLine)*lineLen
        yOut = yInput + np.sin(angleLine)*lineLen

    elif angleLine >= -180 and angleLine < -90:
        angleLine = (180+angleLine)*np.pi/180
        xOut = xInput - np.cos(angleLine)*lineLen
        yOut = yInput - np.sin(angleLine)*lineLen

    elif angleLine <=90 and angleLine >= -90:
        angleLine = angleLine*np.pi/180
        xOut = np.cos(angleLine)*lineLen + xInput
        yOut = np.sin(angleLine)*lineLen + yInput

    elif angleLine > 90 and angleLine <= 180:
        angleLine = (180 - angleLine)*np.pi/180
        xOut = xInput - np.cos(angleLine)*lineLen
        yOut = yInput + np.sin(angleLine)*lineLen

    elif angleLine > 180 and angleLine <= 360:
        angleLine = (angleLine - 180)*np.pi/180
        xOut = xInput - np.cos(angleLine)*lineLen
        yOut = yInput - np.sin(angleLine)*lineLen

    elif angleLine > 360:
        angleLine = (angleLine-360)*np.pi/180
        xOut = np.cos(angleLine)*lineLen + xInput
        yOut = np.sin(angleLine)*lineLen + yInput

    else:
        print('Out of Range')

    return [[xInput,xOut],[yInput,yOut],aOut]


def linkageV(servoA, servoB, xInput, yInput):
    a = planeLinkageReverse([linkageLenA, (linkageLenB+linkageLenC)], -linkageLenD, servoNumCtrl, [0,0], [yInput,xInput])

    [x1,y1,a1] = animateLine(0, 0, linkageLenA, a[0], -90)
    [x2,y2,a2] = animateLine(x1[1],y1[1],(linkageLenB+linkageLenC),a[1],a1+90)
    [x3,y3,a3] = animateLine(x2[1],y2[1],-linkageLenD,a2,90)

    b = middlePosGenOut([linkageLenA, linkageLenB], -a1, a2)

    [x4,y4,a4] = animateLine(x1[1],y1[1],(linkageLenB),a[1],a1+90)

    c = planeLinkageDouble(linkageLenA, [(x4[1]-linkageLenE), y4[1]])

    # pwm.set_pwm(0, 0, init_pwm0+anGen(a[0]))
    # pwm.set_pwm(1, 0, init_pwm1+anGen(c))

    pwm.setPWM(servoA, 0, initPos[servoA]+anGen(a[0])*sc_direction[servoA])
    pwm.setPWM(servoB, 0, initPos[servoA]+anGen(-90+c)*sc_direction[servoB])


def servoDirTest(servoA, angA, servoB, angB):
    pwm.setPWM(servoA, 0, initPos[servoA]+anGen(angA)*sc_direction[servoA])
    pwm.setPWM(servoB, 0, initPos[servoA]+anGen(-90+angB)*sc_direction[servoB])

    print('servo:%d :%d'%(servoA,initPos[servoA]+anGen(angA)*sc_direction[servoA]))
    print('servo:%d :%d'%(servoB,initPos[servoB]+anGen(-90+angB)*sc_direction[servoB]))


def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
    global r
    newline=""
    str_num=str(new_num)
    with open(thisPath+"/PIPPY.py","r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = initial+"%s" %(str_num+"\n")
            newline += line
    with open(thisPath+"/PIPPY.py","w") as f:
        f.writelines(newline)


def configPWM(command_input):
    global  init_pwm0, init_pwm1, init_pwm2, init_pwm3, init_pwm4, init_pwm5, init_pwm6, init_pwm7, init_pwm8, init_pwm9, init_pwm10, init_pwm11, init_pwm12, init_pwm13, init_pwm14, init_pwm15, buffer_8A, buffer_8B, buffer_9A, buffer_9B, buffer_10A, buffer_10B, buffer_11A, buffer_11B

    if 'SiLeft' in command_input:
        numServo = int(command_input[7:])

        if numServo == 0:
            init_pwm0 -= 1

        elif numServo == 1:
            init_pwm1 -= 1

        elif numServo == 2:
            init_pwm2 -= 1

        elif numServo == 3:
            init_pwm3 -= 1


        elif numServo == 4:
            init_pwm4 -= 1

        elif numServo == 5:
            init_pwm5 -= 1

        elif numServo == 6:
            init_pwm6 -= 1

        elif numServo == 7:
            init_pwm7 -= 1


        elif numServo == 8:
            init_pwm8 -= 1

        elif numServo == 9:
            init_pwm9 -= 1

        elif numServo == 10:
            init_pwm10 -= 1

        elif numServo == 11:
            init_pwm11 -= 1


        elif numServo == 12:
            init_pwm12 -= 1

        elif numServo == 13:
            init_pwm13 -= 1

        elif numServo == 14:
            init_pwm14 -= 1

        elif numServo == 15:
            init_pwm15 -= 1

        initServos()


    if 'SiRight' in command_input:
        numServo = int(command_input[8:])

        if numServo == 0:
            init_pwm0 += 1

        elif numServo == 1:
            init_pwm1 += 1

        elif numServo == 2:
            init_pwm2 += 1

        elif numServo == 3:
            init_pwm3 += 1


        elif numServo == 4:
            init_pwm4 += 1

        elif numServo == 5:
            init_pwm5 += 1

        elif numServo == 6:
            init_pwm6 += 1

        elif numServo == 7:
            init_pwm7 += 1


        elif numServo == 8:
            init_pwm8 += 1

        elif numServo == 9:
            init_pwm9 += 1

        elif numServo == 10:
            init_pwm10 += 1

        elif numServo == 11:
            init_pwm11 += 1


        elif numServo == 12:
            init_pwm12 += 1

        elif numServo == 13:
            init_pwm13 += 1

        elif numServo == 14:
            init_pwm14 += 1

        elif numServo == 15:
            init_pwm15 += 1

        initServos()


    if 'PWMMS' in command_input:
        numServo = int(command_input[6:])
        if numServo == 0:
            replace_num('init_pwm0 = ' , init_pwm0)
        elif numServo == 1:
            replace_num('init_pwm1 = ' , init_pwm1)
        elif numServo == 2:
            replace_num('init_pwm2 = ' , init_pwm2)
        elif numServo == 3:
            replace_num('init_pwm3 = ' , init_pwm3)

        elif numServo == 4:
            replace_num('init_pwm4 = ' , init_pwm4)
        elif numServo == 5:
            replace_num('init_pwm5 = ' , init_pwm5)
        elif numServo == 6:
            replace_num('init_pwm6 = ' , init_pwm6)
        elif numServo == 7:
            replace_num('init_pwm7 = ' , init_pwm7)

        elif numServo == 8:
            replace_num('init_pwm4 = ' , init_pwm4)
        elif numServo == 9:
            replace_num('init_pwm5 = ' , init_pwm5)
        elif numServo == 10:
            replace_num('init_pwm6 = ' , init_pwm6)
        elif numServo == 11:
            replace_num('init_pwm7 = ' , init_pwm7)

        elif numServo == 12:
            replace_num('init_pwm12 = ', init_pwm12)
        elif numServo == 13:
            replace_num('init_pwm13 = ', init_pwm13)
        elif numServo == 14:
            replace_num('init_pwm14 = ', init_pwm14)
        elif numServo == 15:
            replace_num('init_pwm15 = ', init_pwm15)

        initServos()


    if 'PWMINIT' == command_input:
        initServos()


    elif 'PWMD' == command_input:
        init_pwm0 = 300
        init_pwm1 = 300
        init_pwm2 = 300
        init_pwm3 = 300

        init_pwm4 = 300
        init_pwm5 = 300
        init_pwm6 = 300
        init_pwm7 = 300

        init_pwm8 = 320
        init_pwm9 = 320
        init_pwm10 = 320
        init_pwm11 = 320

        init_pwm12 = 300
        init_pwm13 = 300
        init_pwm14 = 300
        init_pwm15 = 300

        replace_num('init_pwm0 = ' , init_pwm0)
        replace_num('init_pwm1 = ' , init_pwm1)
        replace_num('init_pwm2 = ' , init_pwm2)
        replace_num('init_pwm3 = ' , init_pwm3)

        replace_num('init_pwm4 = ' , init_pwm4)
        replace_num('init_pwm5 = ' , init_pwm5)
        replace_num('init_pwm6 = ' , init_pwm6)
        replace_num('init_pwm7 = ' , init_pwm7)

        replace_num('init_pwm8 = ' , init_pwm8)
        replace_num('init_pwm9 = ' , init_pwm9)
        replace_num('init_pwm10 = ', init_pwm10)
        replace_num('init_pwm11 = ', init_pwm11)

        replace_num('init_pwm12 = ', init_pwm12)
        replace_num('init_pwm13 = ', init_pwm13)
        replace_num('init_pwm14 = ', init_pwm14)
        replace_num('init_pwm15 = ', init_pwm15)

        initServos()


def anGen(ani):
    return int(round(((ctrlRangeMax-ctrlRangeMin)/angleRange*ani),0))


def linkageD(linkageLen, servoNum, goalPosZ): #E
    sqrtGenOut = np.sqrt(goalPosZ[0]*goalPosZ[0]+goalPosZ[1]*goalPosZ[1])
    nGenOut = (linkageLen[0]*linkageLen[0]+goalPosZ[0]*goalPosZ[0]+goalPosZ[1]*goalPosZ[1]-linkageLen[1]*linkageLen[1])/(2*linkageLen[0]*sqrtGenOut)
    angleA = np.arccos(nGenOut)*180/np.pi

    AB = goalPosZ[1]/goalPosZ[0]

    angleB = np.arctan(AB)*180/np.pi
    angleGenA = angleB - angleA

    return angleGenA*sc_direction[servoNum]


def initServos():
    global initPos

    initPos = [init_pwm0,init_pwm1,init_pwm2,init_pwm3,
               init_pwm4,init_pwm5,init_pwm6,init_pwm7,
               init_pwm8,init_pwm9,init_pwm10,init_pwm11,
               init_pwm12,init_pwm13,init_pwm14,init_pwm15]

    for i in range(0,16):
        pwm.setPWM(i, 0, initPos[i])


def linkageQ(leg, x, y):
    if leg == 1:
        linkageV(0, 1, x, y)
    elif leg == 2:
        linkageV(2, 3, x, y)
    elif leg == 3:
        linkageV(5, 4, x, y)
    elif leg == 4:
        linkageV(7, 6, x, y)


def legStep(s, direc, offset, legName):
    liftD = 0
    if legName == 1 or legName == 3:
        if s == 7:
            liftD = offSetD
        elif s == 8:
            liftD = offSetD/2
        elif s == 9:
            liftD = offSetD/4

    elif legName == 2 or legName == 4:
        if s == 1:
            liftD = offSetD
        elif s == 2:
            liftD = offSetD/2
        elif s == 3:
            liftD == offSetD/4

    if s <= 10 and s > 0:
        x = (walkWiggle/2 - (walkWiggle/9)*(s-1))*direc + offset
        # y = walkHeight + (walkWiggle/2-x)*middleOffset/(walkWiggle/2) - liftD
        y = walkHeight - liftD
    elif s == 11:
        x = -walkWiggle/3*2*direc + offset
        y = walkHeight - liftHeight
    elif s == 12:
        x = walkWiggle/3*2*direc + offset
        y = walkHeight - liftHeight 
    elif s == -1:
        x = offset
        y = walkHeight - liftHeight 

    return x, y


def move(command):
    global sinput

    if command == 'forward':
        leftD  = 1
        rightD = 1
    elif command == 'backward':
        leftD  = -1
        rightD = -1
    elif command == 'left':
        leftD  = -1
        rightD = 1
    elif command == 'right':
        leftD  = 1
        rightD = -1

    if sinput == 1:
        goalPos[0],goalPos[1] = legStep(1,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(10, leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(7,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(4,  rightD, -walkOffset, 4)
    elif sinput == 2:
        goalPos[0],goalPos[1] = legStep(2,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(11, leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(8,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(5,  rightD, -walkOffset, 4)
    elif sinput == 3:
        goalPos[0],goalPos[1] = legStep(3,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(12, leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(9,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(6,  rightD, -walkOffset, 4)
    elif sinput == 4:
        goalPos[0],goalPos[1] = legStep(4,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(1,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(10, rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(7,  rightD, -walkOffset, 4)
    elif sinput == 5:
        goalPos[0],goalPos[1] = legStep(5,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(2,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(11, rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(8,  rightD, -walkOffset, 4)
    elif sinput == 6:
        goalPos[0],goalPos[1] = legStep(6,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(3,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(12, rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(9,  rightD, -walkOffset, 4)
    elif sinput == 7:
        goalPos[0],goalPos[1] = legStep(7,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(4,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(1,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(10, rightD, -walkOffset, 4)
    elif sinput == 8:
        goalPos[0],goalPos[1] = legStep(8,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(5,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(2,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(11, rightD, -walkOffset, 4)
    elif sinput == 9:
        goalPos[0],goalPos[1] = legStep(9,  leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(6,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(3,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(12, rightD, -walkOffset, 4)
    elif sinput == 10:
        goalPos[0],goalPos[1] = legStep(10, leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(7,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(4,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(1,  rightD, -walkOffset, 4)
    elif sinput == 11:
        goalPos[0],goalPos[1] = legStep(11, leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(8,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(5,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(2,  rightD, -walkOffset, 4)
    elif sinput == 12:
        goalPos[0],goalPos[1] = legStep(12, leftD,   walkOffset, 1)
        goalPos[2],goalPos[3] = legStep(9,  leftD,  -walkOffset, 2)
        goalPos[4],goalPos[5] = legStep(6,  rightD,  walkOffset, 3)
        goalPos[6],goalPos[7] = legStep(3,  rightD, -walkOffset, 4)

    sinput += 1
    if sinput > 12:
        sinput = 1


def moveD(command):
    global sinput

    if command == 'forward':
        leftD  = 1
        rightD = 1
    elif command == 'backward':
        leftD  = -1
        rightD = -1
    elif command == 'left':
        leftD  = -1
        rightD = 1
    elif command == 'right':
        leftD  = 1
        rightD = -1
    else:
        return

    if sinput == 1:
        goalPos[0],goalPos[1] = legStep(1,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(7,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(7,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(1,  rightD, -walkOffset, -1)
    elif sinput == 2:
        goalPos[0],goalPos[1] = legStep(2,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(8,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(8,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(2,  rightD, -walkOffset, -1)
    elif sinput == 3:
        goalPos[0],goalPos[1] = legStep(3,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(9,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(9,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(3,  rightD, -walkOffset, -1)
    elif sinput == 4:
        goalPos[0],goalPos[1] = legStep(4,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(10, leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(10, rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(4,  rightD, -walkOffset, -1)
    elif sinput == 5:
        goalPos[0],goalPos[1] = legStep(5,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(11, leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(11, rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(5,  rightD, -walkOffset, -1)
    elif sinput == 6:
        goalPos[0],goalPos[1] = legStep(6,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(12, leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(12, rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(6,  rightD, -walkOffset, -1)
    elif sinput == 7:
        goalPos[0],goalPos[1] = legStep(7,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(1,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(1,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(7,  rightD, -walkOffset, -1)
    elif sinput == 8:
        goalPos[0],goalPos[1] = legStep(8,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(2,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(2,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(8,  rightD, -walkOffset, -1)
    elif sinput == 9:
        goalPos[0],goalPos[1] = legStep(9,  leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(3,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(3,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(9,  rightD, -walkOffset, -1)
    elif sinput == 10:
        goalPos[0],goalPos[1] = legStep(10, leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(4,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(4,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(10, rightD, -walkOffset, -1)
    elif sinput == 11:
        goalPos[0],goalPos[1] = legStep(11, leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(5,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(5,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(11, rightD, -walkOffset, -1)
    elif sinput == 12:
        goalPos[0],goalPos[1] = legStep(12, leftD,   walkOffset, -1)
        goalPos[2],goalPos[3] = legStep(6,  leftD,  -walkOffset, -1)
        goalPos[4],goalPos[5] = legStep(6,  rightD,  walkOffset, -1)
        goalPos[6],goalPos[7] = legStep(12, rightD, -walkOffset, -1)

    sinput += 1
    if sinput > 12:
        sinput = 1


def rangeCtrl(minIn, maxIn, val):
    if val > maxIn:
        val = maxIn
    elif val < minIn:
        val = minIn
    return val


def pitchRoll(pIn, rIn):
    xIn = 0

    y_1 = rangeCtrl(minHeight, maxHeight, middleHeight + pIn - rIn)
    y_2 = rangeCtrl(minHeight, maxHeight, middleHeight - pIn - rIn)
    y_3 = rangeCtrl(minHeight, maxHeight, middleHeight + pIn + rIn)
    y_4 = rangeCtrl(minHeight, maxHeight, middleHeight - pIn + rIn)

    linkageQ(1, xIn, y_1)
    linkageQ(2, xIn, y_2)
    linkageQ(3, xIn, y_3)
    linkageQ(4, xIn, y_4)


def stay(hIn):
    hIn = rangeCtrl(minHeight, maxHeight, hIn)
    linkageQ(1, 0, hIn)
    linkageQ(2, 0, hIn)
    linkageQ(3, 0, hIn)
    linkageQ(4, 0, hIn)


def smove():
    for i in range(0,DPI+1):
        x_1 = lastPos[0] + ((goalPos[0]-lastPos[0])/DPI)*i
        y_1 = lastPos[1] + ((goalPos[1]-lastPos[1])/DPI)*i

        x_2 = lastPos[2] + ((goalPos[2]-lastPos[2])/DPI)*i
        y_2 = lastPos[3] + ((goalPos[3]-lastPos[3])/DPI)*i

        x_3 = lastPos[4] + ((goalPos[4]-lastPos[4])/DPI)*i
        y_3 = lastPos[5] + ((goalPos[5]-lastPos[5])/DPI)*i

        x_4 = lastPos[6] + ((goalPos[6]-lastPos[6])/DPI)*i
        y_4 = lastPos[7] + ((goalPos[7]-lastPos[7])/DPI)*i

        linkageQ(1, x_1, y_1)
        linkageQ(2, x_2, y_2)
        linkageQ(3, x_3, y_3)
        linkageQ(4, x_4, y_4)

        time.sleep(delayTime)

    for i in range(0,8):
        lastPos[i] = goalPos[i]


class PIPPY(threading.Thread):

    def __init__(self, *args, **kwargs):
        super(PIPPY, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()
        self.moveDirection = 'no'
        self.turnDirection = 'no'
        self.commandInput  = 'no'
        self.moveSpeed     = 100

        self.funcMode  = 'no'

        self.xMiddle = 0
        self.yMiddle = 0

        self.mpuDelay= 0.02

        self.pitchValue = 0
        self.rollValue  = 0
        self.initPitch  = 0
        self.initRoll   = 0
        self.valueP     = 0.00025

        self.centerTurn = 1

        stay(middleHeight)


    def pause(self):
        self.__flag.clear()


    def resume(self):
        self.__flag.set()


    def steadyProcessing(self):
        ax, ay = imu.getXY()

        xGet = kfX.kalman(ax)
        yGet = kfY.kalman(ay)

        xDebug = xGet - self.xMiddle
        yDebug = yGet - self.yMiddle
    
        self.pitchValue = rangeCtrl((minHeight - middleHeight), (maxHeight - middleHeight), self.pitchValue + xDebug*self.valueP)
        self.rollValue  = rangeCtrl((minHeight - middleHeight), (maxHeight - middleHeight), self.rollValue - yDebug*self.valueP)
        
        try:
            pitchRoll(self.pitchValue, self.rollValue)
        except:
            pass

        time.sleep(self.mpuDelay)


    def moveStart(self, speed, direction, turning):
        global DPI

        if speed == 100:
            DPI = 1
        elif speed > 50:
            DPI = int(5 - (speed-50)/10)
        elif speed > 40:
            DPI = 1
        else:
            DPI = int(5 - speed/10)

        self.moveSpeed = speed
        self.moveDirection = direction
        self.turnDirection = turning

        if self.turnDirection != 'no':
            self.commandInput = self.turnDirection
        elif self.turnDirection == 'no' and self.moveDirection != 'no':
            self.commandInput = self.moveDirection
        elif self.turnDirection == 'no' and self.moveDirection == 'no':
            self.commandInput = 'no'
            self.moveStop()
            return
        self.resume()


    def moveStop(self):
        self.moveDirection = 'no'
        self.turnDirection = 'no'
        self.deltaSpeed = 11
        self.commandInput = 'no'
        self.pitchValue = self.initPitch
        self.rollValue  = self.initRoll
        stay(walkHeight)


    def functionSelect(self, funcName):
        self.funcMode = funcName
        if self.funcMode == 'no':
            self.moveStop()
            self.pause()
            self.moveStop()
        else:
            self.resume()


    def moveThread(self):
        if selectGait == 0:
            if self.moveSpeed <= speedApart:
                move(self.commandInput)
                smove()
            elif self.moveSpeed > speedApart:
                moveD(self.commandInput)
                smove()
        elif selectGait == 1:
            moveD(self.commandInput)
            smove()
        elif selectGait == 2:
            move(self.commandInput)
            smove()


    def funcProcessing(self):
        if self.funcMode == 'steady':
            self.steadyProcessing()


    def run(self):
        while 1:
            self.__flag.wait()
            if self.funcMode == 'no':
                self.moveThread()
                if self.moveDirection == 'no' and self.turnDirection =='no':
                    self.moveStop()
                    stay(walkHeight)
                    self.pause()
            else:
                self.funcProcessing()
                if self.funcMode == 'no':
                    stay(walkHeight)
                    continue
            # print(self.funcMode)
            pass


if __name__ == '__main__':
    while True:
        # icm20948.icm20948_Gyro_Accel_Read()
        # icm20948.icm20948MagRead()
        # icm20948.icm20948CalAvgValue()
        # time.sleep(0.1)
        # icm20948.imuAHRSupdate(MotionVal[0] * 0.0175, MotionVal[1] * 0.0175,MotionVal[2] * 0.0175,
        #         MotionVal[3],MotionVal[4],MotionVal[5], 
        #         MotionVal[6], MotionVal[7], MotionVal[8])
        # pitch = math.asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3
        # roll  = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3
        # yaw   = math.atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3
        # print("\r\n /-------------------------------------------------------------/ \r\n")
        # print('\r\n Roll = %.2f , Pitch = %.2f , Yaw = %.2f\r\n'%(roll,pitch,yaw))
        # print('\r\nAcceleration:  X = %d , Y = %d , Z = %d\r\n'%(Accel[0],Accel[1],Accel[2]))  
        # print('\r\nGyroscope:     X = %d , Y = %d , Z = %d\r\n'%(Gyro[0],Gyro[1],Gyro[2]))
        # print('\r\nMagnetic:      X = %d , Y = %d , Z = %d'%((Mag[0]),Mag[1],Mag[2]))

        # initServos()
        # time.sleep(1)




        # linkageV(0, 1, 15, 80)
        
        moveD('forward')
        print('1')
        smove()

        
        # for i in range(0,15):
        #   stay(65+i)
        #   time.sleep(delayTime)
        # print('up max')
        # time.sleep(3)

        # for i in range(0,15):
        #   stay(80-i)
        #   time.sleep(delayTime)
        # print('down max')
        # time.sleep(3)
        
    # for i in range(0,60):
    #   linkageQ(4, -20+i, 80)
    #   linkageQ(3, -20+i, 80)
    #   linkageQ(2, -20+i, 80)
    #   linkageQ(1, -20+i, 80)
    #   time.sleep(0.1)
    # for i in range(0,60):
    #   linkageQ(4, 40-i, 80)
    #   linkageQ(3, 40-i, 80)
    #   linkageQ(2, 40-i, 80)
    #   linkageQ(1, 40-i, 80)
    #   time.sleep(0.1)


    # servoDirTest(0, -30, 1, 20)
    # servoDirTest(5, -30, 4, 20)

    # pwm.set_pwm(4, 0, initPos[4]+60)
    # pwm.set_pwm(5, 0, initPos[5]+0)

    # initPos = [init_pwm0,init_pwm1,init_pwm2,init_pwm3,
    #            init_pwm4,init_pwm5,init_pwm6,init_pwm7,
    #            init_pwm8,init_pwm9,init_pwm10,init_pwm11,
    #            init_pwm12,init_pwm13,init_pwm14,init_pwm15]

    # for i in range(0,16):
    #     pwm.set_pwm(i, 0, initPos[i])

    # time.sleep(5)
    # pwm = Adafruit_PCA9685.PCA9685(address=0x60)
    # pwm.set_pwm_freq(50)

    # while 1:
    #     pwm.set_pwm(4, 0, init_pwm0+anGen(-45))
    #     time.sleep(1)
    #     pwm.set_pwm(4, 0, init_pwm0)
    #     time.sleep(1)

    # a = planeLinkageReverse([linkageLenA, (linkageLenB+linkageLenC)], -linkageLenD, servoNumCtrl, [0,0], [80,0])

    # [x1,y1,a1] = animateLine(0, 0, linkageLenA, a[0], -90)
    # [x2,y2,a2] = animateLine(x1[1],y1[1],(linkageLenB+linkageLenC),a[1],a1+90)
    # [x3,y3,a3] = animateLine(x2[1],y2[1],-linkageLenD,a2,90)

    # b = middlePosGenOut([linkageLenA, linkageLenB], -a1, a2)

    # [x4,y4,a4] = animateLine(x1[1],y1[1],(linkageLenB),a[1],a1+90)

    # c = planeLinkageDouble(linkageLenA, [(x4[1]-linkageLenE), y4[1]])

    # # pwm.set_pwm(0, 0, init_pwm0+anGen(a[0]))
    # # pwm.set_pwm(1, 0, init_pwm1+anGen(c))

    # pwm.set_pwm(0, 0, init_pwm0+anGen(a[0])*sc_direction[0])
    # pwm.set_pwm(1, 0, init_pwm0+anGen(-90+c)*sc_direction[1])

# funAlter = Alter()
# funAlter.start()
# pitchRoll(15, 15)

'''
alter = Alter()
alter.start()

alter.functionSelect('steady')
'''

'''
time.sleep(30)

alter.moveAlter(100, 'forward', 'no', 0)

time.sleep(3)

alter.moveStop()
'''

'''
screen = OLED()
screen.start()

startBreathLight(92, 128, 255)

screen.oledShowText('SBD', 0, 0)
frontLightCtrl('on')

alter.moveAlter(100, 'forward', 'no', 0)
time.sleep(3)
alter.moveStop()

# screen.showLooks('laugh')
lightStop()
set2812(35,35,35)
time.sleep(1)
frontLightCtrl('off')

alter.moveAlter(100, 'backward', 'left', 0)
time.sleep(3)
alter.moveStop()

startPoliceLight()
time.sleep(1)

alter.moveAlter(40, 'backward', 'no', 0)
time.sleep(3)
alter.moveStop()

set2812(0,0,0)
time.sleep(1)
# screen.showLooks('laugh')
alter.moveAlter(40, 'forward', 'no', 0)
time.sleep(2)
alter.moveStop()

lightStop()
'''

# while 1:
#   linkageQ(1, walkWiggle/2, walkHeight)
#   time.sleep(1)
#   linkageQ(1, -walkWiggle/2, walkHeight)
#   time.sleep(1)
#   linkageQ(1, 0, walkHeight-liftHeight)
#   time.sleep(1)
#   pass

# initServos()

# while 1:
#   moveD('forward')
#   smove()
    # print('loop')
    # pass

# while 1:
#   forDPI()
#   pass

# while 1:
#   for i in range(0,20):
#       linkageQ(2,(-10),65-i)
#       linkageQ(1,(25),65-i)

#       time.sleep(0.02)

#   for i in range(0,20):
#       linkageQ(2,(-10),45+i)
#       linkageQ(1,(25),45+i)

#       time.sleep(0.02)


# while 1:
#   for i in range(0,20):
#       linkageQ(2,(25),65-i)
#       linkageQ(1,(25),65-i)
#       linkageQ(3,(25),65-i)
#       linkageQ(4,(25),65-i)
#       time.sleep(0.02)

#   for i in range(0,20):
#       linkageQ(2,(25),45+i)
#       linkageQ(1,(25),45+i)
#       linkageQ(3,(25),45+i)
#       linkageQ(4,(25),45+i)
#       time.sleep(0.02)


# while 1:
#   for i in range(0,60):
#       a = linkageD(linkageDInput, 1, [60,30-i])
#       pwm.set_pwm(1,0,init_pwm1 + anGen(a))
#       time.sleep(0.02)

# while 1:
#   for i in range(0,60):
#       a = linkageD(linkageDInput, 0, [60,-30+i])
#       pwm.set_pwm(0,0,init_pwm0 + anGen(a))
#       time.sleep(0.01)

# a = anGen(90)
# b = anGen(0)

# while 1:
#   pwm.set_pwm(0,0,init_pwm0 + a)
#   print(a)
#   time.sleep(1)
#   pwm.set_pwm(0,0,init_pwm0 + b)
#   print(b)
#   time.sleep(1)