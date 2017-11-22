from collections import deque
import cv2
import serial
import numpy as np
from driveTest import DriveTest
import math
import threading

dist = 0.115
wheelone = 0
wheeltwo = 120
wheelthree = 240

# ser = serial.Serial('COM3',baudrate=115200,timeout = 0.8,dsrdtr=True)
# qprint(ser.isOpen())

# ser = serial.Serial('COM3',baudrate=115200,timeout = 0.01,dsrdtr=True)

gameIsOn = False

f = open("RFinfo.txt")
field = f.readline().split("=")[1].split("#")[0].strip()
robotChar = f.readline().split("=")[1].split("#")[0].strip()
n = 'rf:a' + field + robotChar


# n = 'rf:aAB'


def readin(filename):
    read = open(filename, "r")
    f = read.readlines()
    algne = f[0].split(",")
    if len(algne) == 0:
        return [0, 0, 0], [179, 255, 255]
    alam = []
    korgem = []
    x = 0
    read.close()
    for i in algne:
        if x == 0:
            alam.append(int(i))
            x = 1
        else:
            korgem.append(int(i))
            x = 0
    return np.array(alam), np.array(korgem)


##vana 20.11.2017
knownWidth = 16
# focallength = (64 * 128) / knownWidth
distBuffer = deque()
focallength = (59 * 151) / knownWidth

# define the lower and upper boundaries of the
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower, greenUpper = readin("Pall.txt")
# greenLower = (40, 46, 58)
# greenUpper = (78, 88, 206)
# 5,38,255,255,43,160
# VANAD
teamPink = False
if teamPink:  # Attacking blue basket
    basketLower, basketUpper = readin("Varavsinine.txt")
    # basketLower = (100, 79, 124)
    # basketUpper = (108, 145, 191)
else:  # Attacking pink basket
    basketLower, basketUpper = readin("VaravLilla.txt")
    # basketLower = (0, 52, 144)
    # basketUpper = (57, 79, 209)
# blueLower = (30, 90, 52)
# blueUpper = (47, 139, 76)

# pallKeskel = False
basketIsMiddle = False
soidanOtse = False
circlingBall = False
makeThrow = False
# stopLoop = False
keskX = 315
basketSmall = 300
basketLarge = 312
counter = 0
# gameOn = False
basketIsLeft = -1

pts = deque()
img = np.zeros((480, 640, 3), np.uint8)


def drawThing(cnts, isBall):
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    pindala = cv2.contourArea(c)
    # print(pindala)

    if isBall:
        if pindala < 30:
            return -1, -1
    else:
        if pindala < 600:
            return -1, -1, -1, -1
    if isBall:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # print("Raadius: " + str(radius))
            # only proceed if the radius meets a minimum size
            if radius > 3:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points

                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, str(center), center, cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
                cv2.putText(frame, str(round((radius ** 2) * 3.14)), (center[0] + 200, center[1]),
                            cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
        return x, y
    else:
        # x, y, w, h = cv2.boundingRect(c)

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # if w > 5:
        rotation = rect[2]
        if rotation < -10:
            h = rect[1][0]
            w = rect[1][1]
        else:
            h = rect[1][1]
            w = rect[1][0]
        x = rect[0][0]
        y = rect[0][1]

        # qprint (x,y,w,h)
        if w > 5:
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
            cv2.putText(frame, "Laius: " + str(round(w)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
        return x, y, w, h
        # return 0, 0, 0, 0


# grab the reference to the webcam
camera = cv2.VideoCapture(0)
kernel = np.ones((2, 2), np.uint8)
kernelBasket = np.ones((4, 4), np.uint8)


def ballMiddle(x):
    if x >= 297 and x < 315:
        return True
    else:
        return False


def moveSpeed(bally):
    return max(70 - int(bally / 6), 20)


def isMiddle(x):
    if x >= basketSmall and x < basketLarge:
        # if x >= 315 and x < 325:
        # if x >= 300 and x < 320:
        return True
    else:
        return False


def findAngle(x, y):
    return int(math.degrees(math.atan((keskX - x) / y)) + 90)  # 313 mootmiste tulemusena leitud keskmine


def throwStrength(distance):
    return 1165 + distance
    """if distance <= 110:
        #algvaartus 1232
        distance_ = distance - 70 + 1222
        print ("Kaugus + tugevus: " + str(distance) + " " + str(distance_))
        return distance_
    elif distance >= 111:
        #algvaartus 1083
        distance___ = 1063 + distance * 1.6
        print ("Kaugus + tugevus: " + str(distance) + " " + str(distance___))
        return distance___"""


drive = DriveTest()
rfser = drive.ser
print("Press 'p' to start: ")
print(n)
while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    # erode + dilate asemel saab kasutada opening.

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # mask = cv2.erode(mask, None, iterations=1)
    # mask = cv2.dilate(mask, None, iterations=1)

    maskBlue = cv2.inRange(hsv, basketLower, basketUpper)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_CLOSE, kernelBasket)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_OPEN, kernelBasket)
    maskCombo = cv2.add(mask, maskBlue)
    cv2.imshow("combo", maskCombo)
    # maskBlue = cv2.erode(maskBlue, None, iterations=1)
    # maskBlue = cv2.dilate(maskBlue, None, iterations=1)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cntsPurple = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(distBuffer) > 30:
        distBuffer.popleft()
    ballIsMiddle = False
    basketIsMiddle = False
    bothMiddle = False
    # print("COunter"+str(counter))
    if counter > 60:
        counter = 0
        makeThrow = False
        drive.stopThrow()
        # gameOn = False
        # drive.shutdown()
    if len(cntsPurple) > 0:
        basketx, baskety, w, h = drawThing(cntsPurple, False)
        cv2.putText(frame, str(basketx + int(w / 2)), (10, 370), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
        # lisatav number on korvi laius(vaja kontrollida, kas on ikka 8)
        distance = 8 + (knownWidth * focallength / w)
        if distance > 0:
            distBuffer.append(distance)
        cv2.putText(frame, "Kaugus: " + str(distance), (10, 200), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
        cv2.putText(frame, "Pindala: " + str(cv2.contourArea(max(cntsPurple, key=cv2.contourArea))), (10, 250),
                    cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
    speed = 1000
    if drive.gameOn:
        if makeThrow:  # If we have spun around the ball and are going to throw
            counter += 1
            if len(cntsPurple) > 0:
                # basketx, baskety, w, h = drawThing(cntsPurple, False)
                drive.setspeed(90, 10)
                if counter == 1:
                    if len(distBuffer) > 0:
                        mindist = min(distBuffer)
                        print(mindist)
                        if mindist > 0:
                            speed = throwStrength(mindist)
                    drive.startThrow(speed)
        elif len(cnts) > 0 and cv2.contourArea(max(cnts, key=cv2.contourArea)) > 30: #Siia lisada tingimus and isOnField()
            # print(len(cnts))
            ballx, bally = drawThing(cnts, True)
            angle = findAngle(ballx, bally)
            # print(str(pallx)+" "+ str(pally)+" "+str(nurk))
            # print(ballx)
            ballIsMiddle = ballMiddle(ballx)

            if circlingBall:  # When we are spinning around the ball
                if len(cntsPurple) > 0:
                    if basketx < keskX:
                        drive.circleBallRight()
                    else:
                        drive.circleBallLeft()
                    basketIsMiddle = isMiddle(basketx)

                    # print (basketx + int(w/2))
                    ballIsMiddle = ballMiddle(ballx)
                    # print("pall: "+str(ballIsMiddle))
                    # print("korv: "+str(basketIsMiddle))
                    if basketIsMiddle:
                        drive.shutdown()
                        # print("Korv ja pall keskel")
                        circlingBall = False
                        makeThrow = True
                        # gameOn = False
                        drive.stopThrow()


                else:
                    drive.circleBallLeft()
            elif bally > 400:  # If we are close enough to the ball after approaching it
                if ballIsMiddle and not makeThrow:
                    # drive.shutdown()
                    drive.circleBallLeft()
                    circlingBall = True
                else:
                    if ballx < 307:
                        drive.setspeed(180, 10)
                    else:
                        drive.setspeed(0, 10)

            else:
                #drive.setspeed(angle, 30)
                drive.setspeed(angle, moveSpeed(bally))
        elif counter == 0:
            drive.spinleft()

        if ballIsMiddle & basketIsMiddle:
            bothMiddle = True
            cv2.putText(frame, "Molemad on keskel! :O", (10, 370), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
    center = None
    cv2.line(frame, (keskX, 0), (keskX, 480), (255, 0, 0), 1)
    cv2.line(frame, (basketSmall, 0), (basketSmall, 480), (255, 0, 0), 1)
    cv2.line(frame, (basketLarge, 0), (basketLarge, 480), (255, 0, 0), 1)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        ##cv2.imwrite("test.png", frame)
        drive.shutdown()
        drive.stopThrow()
        drive.running = False
        stopLoop = True
        break
    # if the 'p' key is pressed, pause/unpause robot game logic
    elif key == ord("p"):
        drive.gameOn = not drive.gameOn
        drive.shutdown()
        drive.stopThrow()
drive.running = False
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
