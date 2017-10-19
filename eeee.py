from collections import deque
import cv2
import serial
import numpy as np
import driveTest as drive
import math

dist = 0.115
wheelone = 0
wheeltwo = 120
wheelthree = 240


#ser = serial.Serial('COM3',baudrate=115200,timeout = 0.8,dsrdtr=True)
#qprint(ser.isOpen())


# define the lower and upper boundaries of the
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (49, 35, 72)
greenUpper = (80, 108, 204)
#greenLower = (0, 113, 126)
#greenUpper = (13, 196, 255)
blueLower = (101, 91, 42)
blueUpper = (122, 185, 129)
#pallKeskel = False
korvKeskel = False
sõidanOtse = False

pts = deque()
img = np.zeros((480, 640, 3), np.uint8)
def joonistaAsi(cnts):

    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    pindala = cv2.contourArea(c)
    print(pindala)
    if pindala < 70:
        return -1,-1
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    if M["m00"] > 0:
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print("Raadius: "+ str(radius))
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

#grab the reference to the webcam
camera = cv2.VideoCapture(1)
kernel = np.ones((5,5), np.uint8)


def kasKeskel(x):
    if x >= 307 and x < 317:
        return True
    else:
        return False
def leiaNurk(x, y):
    return int(math.degrees(math.atan((313-x)/y))+90) # 313 mõõtmiste tulemusena leitud keskmine



while True:
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

    pallKeskel = False
    korvKeskel = False
    mõlemadKeskel = False
    if len(cnts) > 0:
        #print(len(cnts))
        pallx, pally = joonistaAsi(cnts)
        nurk = leiaNurk(pallx,pally)
        #print(str(pallx)+" "+ str(pally)+" "+str(nurk))
        print(pallx)
        pallKeskel = kasKeskel(pallx)

        if pally > 400:
            if pallKeskel:
                drive.shutdown()
            else:
                if pallx < 307:
                    drive.setspeed(180, 10)
                else:
                    drive.setspeed(0, 10)

        else:
            drive.setspeed(nurk, 20)
        """
        if pallx ==-1:
            drive.spinleft()
        elif pallKeskel:
            print(pallKeskel)
            if not sõidanOtse:
                drive.shutdown()
                sõidanOtse = True
                drive.setspeed(90)
            cv2.putText(mask, "Pall on keskel!", (10, 330), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV2GRAY_420)
        else:
            sõidanOtse = False
            pallKeskel = False

            if pallx < 300:
                drive.spinright()

            elif pallx> 340:
                drive.spinleft()
    """
    else:
        pass
        #drive.spinleft()

    """if len(cntsPurple) > 0:
        korvx, korvy = joonistaAsi(cntsPurple)
        korvKeskel = kasKeskel(korvx)
        if korvKeskel:
            cv2.putText(frame, "Korv on keskel!", (10, 350), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
    """
    if pallKeskel & korvKeskel:
        mõlemadKeskel = True
        cv2.putText(frame, "Molemad on keskel! :O", (10, 370), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
    center = None
    cv2.line(frame,(313,0),(313,480),(255,0,0), 1)

    # update the points queue
    pts.appendleft(center)
    # show the frame to our screen
    cv2.imshow("Frame", frame  )
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        ##cv2.imwrite("test.png", frame)
        drive.shutdown()
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()