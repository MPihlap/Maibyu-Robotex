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


#ser = serial.Serial('COM3',baudrate=115200,timeout = 0.8,dsrdtr=True)
#qprint(ser.isOpen())

#ser = serial.Serial('COM3',baudrate=115200,timeout = 0.01,dsrdtr=True)
gameIsOn = False
n = 'rf:aAB'

# define the lower and upper boundaries of the
# ball in the HSV color space, then initialize the
# list of tracked points
#greenLower = (49, 35, 72)
#greenUpper = (80, 108, 204)
greenLower = (5, 255, 43)
greenUpper = (38, 255, 144)
#5,38,255,255,43,160
blueLower = (24, 81, 41)
blueUpper = (110, 255, 96)
#pallKeskel = False
basketIsMiddle = False
soidanOtse = False
circlingBall = False
makeThrow = False
keskX = 313
counter = 0

pts = deque()
img = np.zeros((480, 640, 3), np.uint8)
def drawThing(cnts, isBall):

    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    pindala = cv2.contourArea(c)
    #print(pindala)
    if pindala < 70:
        if isBall:
            return -1,-1
        else:
            return -1, -1, -1, -1
    if isBall:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #print("Raadius: "+ str(radius))
        # only proceed if the radius meets a minimum size
            if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points

                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, str(center), center, cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
                cv2.putText(frame, str(round((radius**2)*3.14)), (center[0]+200,center[1]), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
        return x,y
    else:
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if w > 5:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, "Laius: " + str(round(w)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
            return x, y, w, h
        #return 0, 0, 0, 0
#grab the reference to the webcam
camera = cv2.VideoCapture(0)
kernel = np.ones((5,5), np.uint8)
def isMiddle(x):
    #if x >= 307 and x < 317:
    if x >= 300 and x < 320:
        return True
    else:
        return False





def findAngle(x, y):
    return int(math.degrees(math.atan((keskX - x) / y)) + 90) # 313 mootmiste tulemusena leitud keskmine



"""while True:
    vastus = ser.read(19).decode("utf-8")
    if vastus == '<ref:aAXSTART---->\n':
        ser.write(n + 'ACK-----')
        print("sain start kasu")
        gameIsOn = True
    elif vastus == '<ref:PING----->\n':
        print("sain ping kasu")
        ser.write(n + 'ACK-----')
    gameIsOn = True
    if gameIsOn:
"""
drive = DriveTest()
while True:
    """vastus = ser.read(12)
    if vastus == '<ref:aAXSTOP------\n>':
        ser.write(n + 'ACK-----')
        print("sain stop kasu")
        gameIsOn = False
        break
        """
    # grab the current frame
    (grabbed, frame) = camera.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    # erode + dilate asemel saab kasutada opening.

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #mask = cv2.erode(mask, None, iterations=1)
    #mask = cv2.dilate(mask, None, iterations=1)

    maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
    maskBlue = cv2.morphologyEx(maskBlue,cv2.MORPH_CLOSE,kernel)
    #maskBlue = cv2.erode(maskBlue, None, iterations=1)
    #maskBlue = cv2.dilate(maskBlue, None, iterations=1)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cntsPurple = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[-2]

    ballIsMiddle = False
    basketIsMiddle = False
    bothMiddle = False

    if counter > 60:
        counter = 0
        makeThrow = False
        drive.throwSpeed = 1000
    if len(cntsPurple) > 0:
        basketx, baskety, w, h = drawThing(cntsPurple, False)
    if len(cnts) > 0:
        #print(len(cnts))
        ballx, bally = drawThing(cnts, True)
        angle = findAngle(ballx, bally)
        #print(str(pallx)+" "+ str(pally)+" "+str(nurk))
        #print(ballx)
        ballIsMiddle = isMiddle(ballx)

        if makeThrow: # If we have spun around the ball and are going to throw
            counter += 1
            if len(cntsPurple) > 0:
                basketx, baskety, w, h = drawThing(cntsPurple, False)
                drive.setspeed(90, 10)
                drive.startThrow(int(1600 - (w -35)*(-4)))
            # lisa counter, et saaks makeThrow-st valja.

        elif circlingBall:          # When we are spinning around the ball
            if len(cntsPurple) > 0:
                basketIsMiddle = isMiddle(basketx)
                ballIsMiddle = isMiddle(ballx)
                #print("pall: "+str(ballIsMiddle))
                print("korv: "+str(basketIsMiddle))
                if basketIsMiddle:
                    drive.shutdown()
                    print("Korv ja pall keskel")
                    circlingBall = False
                    makeThrow = True
            else:
                drive.circleBallLeft()
        elif bally > 400:   # If we are close enough to the ball after approaching it
            if ballIsMiddle and not makeThrow:
                if basketx < keskX:
                    drive.circleBallRight()
                else:
                    drive.circleBallLeft()
                #drive.shutdown()
                #drive.circleBallLeft()
                circlingBall = True
            else:
                if ballx < 307:
                    drive.setspeed(180, 10)
                else:
                    drive.setspeed(0, 10)

        else:
            drive.setspeed(angle, 20)
        """
        if pallx ==-1:
            drive.spinleft()
        elif pallKeskel:
            print(pallKeskel)
            if not soidanOtse:
                drive.shutdown()
                soidanOtse = True
                drive.setspeed(90)
            cv2.putText(mask, "Pall on keskel!", (10, 330), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV2GRAY_420)
        else:
            soidanOtse = False
            pallKeskel = False

            if pallx < 300:
                drive.spinright()

            elif pallx> 340:
                drive.spinleft()
    """
    else:
        drive.spinleft()

    """if len(cntsPurple) > 0:
        korvx, korvy = joonistaAsi(cntsPurple)
        korvKeskel = kasKeskel(korvx)
        if korvKeskel:
            cv2.putText(frame, "Korv on keskel!", (10, 350), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
    """
    if ballIsMiddle & basketIsMiddle:
        bothMiddle = True
        cv2.putText(frame, "Molemad on keskel! :O", (10, 370), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
    center = None
    cv2.line(frame, (keskX, 0), (keskX, 480), (255, 0, 0), 1)

    # update the points queue
    pts.appendleft(center)
    # show the frame to our screen
    cv2.imshow("Frame", frame  )
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        ##cv2.imwrite("test.png", frame)
        drive.shutdown()
        drive.stopThrow()
        drive.running = False
        break
drive.running = False
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()