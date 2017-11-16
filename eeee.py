
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

knownWidth = 16
focallength = (64*128)/knownWidth

# define the lower and upper boundaries of the
# ball in the HSV color space, then initialize the
# list of tracked points
#greenLower = (49, 35, 72)
#greenUpper = (80, 108, 204)
#viimased
#greenLower = (5, 255, 43)
#greenUpper = (38, 255, 144)
greenLower = (21,75,144)
greenUpper = (34,150,243)
#5,38,255,255,43,160
#VANAD
blueLower = (0, 11, 100)
blueUpper = (179, 71, 181)
#blueLower = (30, 90, 52)
#blueUpper = (47, 139, 76)

#pallKeskel = False
basketIsMiddle = False
soidanOtse = False
circlingBall = False
makeThrow = False
keskX = 320
counter = 0
gameOn = False
basketIsLeft = -1

#blueLower = (24, 71, 33)
#blueUpper = 44, 134, 49

pts = deque()
img = np.zeros((480, 640, 3), np.uint8)
def drawThing(cnts, isBall):

    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    pindala = cv2.contourArea(c)
    #print(pindala)
    if pindala < 15:
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
        #x, y, w, h = cv2.minAreaRect(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #if w > 5:
        rotation = rect[2]
        if rotation < -10:
            h = rect[1][0]
            w = rect[1][1]
        else:
            h = rect[1][1]
            w = rect[1][0]
        x = rect[0][0]
        y = rect[0][1]


        #qprint (x,y,w,h)
        #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
        cv2.putText(frame, "Laius: " + str(round(w)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
        return x, y, w, h
        #return 0, 0, 0, 0
#grab the reference to the webcam
camera = cv2.VideoCapture(0)
kernel = np.ones((2,2), np.uint8)
def ballMiddle(x):
    if x >= 310 and x < 330:
        return True
    else:
        return False

def isMiddle(x):
    #if x >= 310 and x < 316:
    if x >= 310 and x < 330:
    #if x >= 300 and x < 320:
        return True
    else:
        return False





def findAngle(x, y):
    return int(math.degrees(math.atan((keskX - x) / y)) + 90) # 313 mootmiste tulemusena leitud keskmine

def throwStrength(distance):
    if distance <= 110:
        distance_ = distance - 70 + 1232
        print ("Kaugus + tugevus: "+ str(distance) + " "+ str(distance_))
        return distance_
    elif distance >= 111:
        distance___ = 1090 + distance * 1.6
        print ("Kaugus + tugevus: "+ str(distance) + " "+ str(distance___))
        return distance___

drive = DriveTest()
#rfser = drive.ser
print ("Press 'p' to start: ")

"""while True:
    vastus = str(rfser.read(19))
    if vastus == '<ref:aAXSTART---->\n' or vastus == '<ref:aABSTART---->\n':
        rfser.write(n + 'ACK-----')
        print("sain start kasu")
        gameOn = True

    elif vastus == '<ref:aAXSTOP----->\n':
        rfser.write(n + 'ACK-----')
        print("sain stop kasu")
    elif vastus == '<ref:aABPING----->\n':
        print("sain ping kasu")
        rfser.write(n + 'ACK-----')

    if gameOn:
        while True:
            vastus = rfser.read(19)
            if vastus == '<ref:aAXSTOP----->\n':
                rfser.write(n + 'ACK-----')
                print("sain stop kasu")
                gameIsOn = False
                break
"""
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

    #mask = cv2.erode(mask, None, iterations=1)
    #mask = cv2.dilate(mask, None, iterations=1)

    maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
    maskBlue = cv2.morphologyEx(maskBlue,cv2.MORPH_OPEN,kernel)
    maskCombo = cv2.add(mask,maskBlue)
    cv2.imshow("combo",maskCombo)
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
    #print("COunter"+str(counter))
    if counter > 60:
        counter = 0
        makeThrow = False
        drive.stopThrow()
        #gameOn = False
        #drive.shutdown()
    if len(cntsPurple) > 0:
        basketx, baskety, w, h = drawThing(cntsPurple, False)
        # lisatav number on korvi laius(vaja kontrollida, kas on ikka 8)
        distance = 8 + (knownWidth * focallength / w)
        cv2.putText(frame, "Kaugus: " + str(distance), (10, 200), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)

    if gameOn:
        if makeThrow:  # If we have spun around the ball and are going to throw
            counter += 1
            if len(cntsPurple) > 0:
                # basketx, baskety, w, h = drawThing(cntsPurple, False)
                drive.setspeed(90, 10)
                if counter == 1:
                    drive.startThrow(throwStrength(distance))
        elif len(cnts) > 0 and cv2.contourArea(max(cnts, key= cv2.contourArea)) > 70:
            print(len(cnts))
            ballx, bally = drawThing(cnts, True)
            angle = findAngle(ballx, bally)
            #print(str(pallx)+" "+ str(pally)+" "+str(nurk))
            #print(ballx)
            ballIsMiddle = ballMiddle(ballx)



            if circlingBall:          # When we are spinning around the ball
                if len(cntsPurple) > 0:
                    if basketx + int(w/2) < keskX:
                        drive.circleBallRight()
                    else:
                        drive.circleBallLeft()
                    basketIsMiddle = isMiddle(basketx + int(w/2))
                    #print (basketx + int(w/2))
                    ballIsMiddle = ballMiddle(ballx)
                    #print("pall: "+str(ballIsMiddle))
                    #print("korv: "+str(basketIsMiddle))
                    if basketIsMiddle:
                        drive.shutdown()
                        #print("Korv ja pall keskel")
                        circlingBall = False
                        makeThrow = True
                        #gameOn = False
                        drive.stopThrow()


                else:
                    drive.circleBallLeft()
            elif bally > 400:   # If we are close enough to the ball after approaching it
                if ballIsMiddle and not makeThrow:
                    #drive.shutdown()
                    drive.circleBallLeft()
                    circlingBall = True
                else:
                    if ballx < 307:
                        drive.setspeed(180, 10)
                    else:
                        drive.setspeed(0, 10)

            else:
                drive.setspeed(angle, 20)
        elif counter == 0:
            drive.spinleft()

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
    #if the 'p' key is pressed, pause/unpause robot game logic
    elif key == ord("p"):
        gameOn = not gameOn
        drive.shutdown()
        drive.stopThrow()
drive.running = False
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
