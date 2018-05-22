from collections import deque
from MainboardComms import Mainboard
from UtilityFunctions import *
from ImageProcessing import *
import cv2
import serial
import numpy as np
import math

teamPink = True # Attacking blue basket
setOpposingBasketThresh(teamPink)

gameIsOn = False
f = open("RFinfo.txt")
field = f.readline().split("=")[1].split("#")[0].strip()
robotChar = f.readline().split("=")[1].split("#")[0].strip()

n = 'rf:a' + field + robotChar
knownWidth = 16
distBuffer = deque()
focallength = (59 * 151) / knownWidth
throwStrengths = sorted(readThrowStr("visketugevused.csv"))
teamPink = True
moveMid = False
startLimit = 75
Kp = 0.1
Ki = 0.00
Kd = 0.02
previous_error = 0
integral = 0
MAX_SPEED = 20
MIN_SPEED = 6
circlingBall = False
makeThrow = False
keskX = 307
basketSmall = keskX - 7
basketLarge = keskX + 7
counter = 0
startCounter = 0
basketIsLeft = None

camera = cv2.VideoCapture(0)

drive = Mainboard()
rfser = drive.ser
def ballMiddle(x):
    return keskX + 3 > x >= keskX - 3

def moveSpeed(bally):
    return max(90 - int(bally / 5), 20)

def isMiddle(x):
    return basketLarge > x >= basketSmall

def findAngle(x, y):
    return int(math.degrees(math.atan((keskX - x) / y)) + 90)  # 313 mootmiste tulemusena leitud keskmine

def pause():
    drive.gameOn = not drive.gameOn
    drive.shutdown()
    drive.stopThrow()

def throwStrength(distance):
    distance = distance - 10
    for pair in throwStrengths:
        if distance == pair[0]:
            print("found equal distance")
            return pair[1]
        if distance < pair[0] and throwStrengths.index(pair) != 0:
            previous = throwStrengths[throwStrengths.index(pair) - 1]
            return previous[1] + (pair[1] - previous[1]) / 2
        elif throwStrengths.index(pair) == 0 and distance < pair[0]:
            return throwStrengths[0][1]
    return 1600
print("Press 'p' to start: ")

def calculateSpeedWithPID(objDistance):
    global integral
    error = abs(keskX - objDistance)
    integral += error
    speed = Kp * error + Ki * integral
    print(speed)
    if speed > MAX_SPEED:
        return MAX_SPEED
    elif speed < MIN_SPEED:
        return MIN_SPEED
    return speed


while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    ballCnts, basketCnts = contourify(frame)

    if len(distBuffer) > 15:
        distBuffer.popleft()
    ballIsMiddle = False
    basketIsMiddle = False
    bothMiddle = False
    ballx = -1
    basketx = -1

    #toimub ainult viskel
    if counter > 60:
        counter = 0
        makeThrow = False
        drive.stopThrow()
        distBuffer.clear()

    #toimub iga kord
    if len(basketCnts) > 0:
        basketx, baskety, w, h = drawThing(frame, basketCnts, False)
        if basketx > 0:
            basketIsLeft = basketx < keskX

        cv2.putText(frame, "basketX " + str(basketx), (10, 370), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
        # lisatav number on korvi laius(vaja kontrollida, kas on ikka 8)
        distance = (knownWidth * focallength / w) # + 8
        if distance > 0:
            distBuffer.append(distance)
        cv2.putText(frame, "Kaugus: " + str(distance), (10, 200), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
        cv2.putText(frame, "Pindala: " + str(cv2.contourArea(max(basketCnts, key=cv2.contourArea))), (10, 250),
                    cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
    if len(ballCnts) > 0:
        ballx, bally = drawThing(frame, ballCnts, True)
        angle = findAngle(ballx, bally)
        ballIsMiddle = ballMiddle(ballx)
    speed = 1000
    if drive.gameOn:
        if startCounter < startLimit and moveMid:
            drive.setspeed(90, 50)
            startCounter += 1
        elif makeThrow:  # If we have spun around the ball and are going to throw
            counter += 1
            drive.setspeed(90, 10)
            if counter == 1:
                if len(distBuffer) > 0:
                    maxdist = max(distBuffer)
                    if maxdist > 0:
                        speed = throwStrength(maxdist)
                drive.startThrow(speed)
        elif ballx != -1: # If ball is detected
            if circlingBall:  # When we are spinning around the ball
                if bally < 400:
                    circlingBall = False
                if basketx != -1: # If basket is detected
                    if basketx < keskX:
                        speed = calculateSpeedWithPID(basketx)
                        drive.circleBallRight(speed)
                    else:
                        speed = calculateSpeedWithPID(basketx)
                        drive.circleBallLeft(speed)
                    basketIsMiddle = isMiddle(basketx)
                    ballIsMiddle = ballMiddle(ballx)
                    if basketIsMiddle:
                        if ballIsMiddle:
                            drive.shutdown()
                            circlingBall = False
                            makeThrow = True
                            drive.stopThrow()
                        else:
                            if ballx < keskX:
                                drive.setspeed(180, 3)
                            else:
                                drive.setspeed(0, 3)
                else:   # If basket is not visible
                    if basketIsLeft:
                        drive.circleBallRight(20)
                    else:
                        drive.circleBallLeft(20)
            elif bally > 400:  # If we are close enough to the ball after approaching it
                if ballIsMiddle and not makeThrow:
                    drive.circleBallLeft(9)
                    circlingBall = True
                else:
                    if ballx < keskX:
                        drive.setspeed(180, 10)
                    else:
                        drive.setspeed(0, 10)

            else:
                drive.setspeed(angle, moveSpeed(bally))
        elif counter == 0:
            drive.spinright()

        if ballIsMiddle & basketIsMiddle:
            bothMiddle = True
            cv2.putText(frame, "Molemad on keskel! :O", (10, 370), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
    center = None
    cv2.line(frame, (keskX, 0), (keskX, 480), (255, 0, 0), 1)
    cv2.line(frame, (basketSmall, 0), (basketSmall, 480), (255, 0, 0), 1)
    cv2.line(frame, (basketLarge, 0), (basketLarge, 480), (255, 0, 0), 1)
    cv2.line(frame, (0 ,400),(640,400),(0,0,255),1)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    #print key

    # if the 'q' key is pressed, stop the loop
    wasdControl(key, drive)
    if key == ord("."):
        print (basketIsLeft)
    if key == ord("q"):
        ##cv2.imwrite("test.png", frame)
        drive.shutdown()
        drive.stopThrow()
        drive.running = False
        stopLoop = True
        break
    # if the 'p' key is pressed, pause/unpause robot game logic
    elif key == ord("p"):
        pause()
drive.running = False
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()