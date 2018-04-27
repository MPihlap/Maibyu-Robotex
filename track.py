from collections import deque
import numpy as np
import cv2

from MainboardComms import DriveTest
# define the lower and upper boundaries of the
# ball in the HSV color space, then initialize the
# list of tracked points
#greenLower = (49, 35, 72)
#greenUpper = (80, 108, 204)
#greenLower = (0, 113, 126)
#greenUpper = (13, 196, 255)
greenUpper = (27,255,62)
greenLower = (40,255,173)
#blueLower = (24,81,41)  #
#blueUpper = (110,255,96)
blueLower = (30, 90, 52)
blueUpper = (47, 139, 76)
#blueLower = (101,107,94)
#blueUpper = (115,147,134)
pts = deque()

#grab the reference to the webcam
camera = cv2.VideoCapture(0)
#drive = DriveTest()
kernel = np.ones((2, 2), np.uint8)
kernelBasket = np.ones((4, 4), np.uint8)
while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the m((x, y), radius) = cv2.minEnclosingCircle(c)
    maskBall = cv2.inRange(hsv, greenLower, greenUpper)
    maskBall = cv2.morphologyEx(maskBall, cv2.MORPH_OPEN, kernel)
    cnts = cv2.findContours(maskBall, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
    #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # only proceed if the radius meets a minimum size
    #radius = 500
    #if radius > 0:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points

        #cv2.circle(frame, (int(x), int(y)), int(radius),
        #           (0, 255, 255), 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        #cv2.circle(frame, center, 5, (0, 0, 255), -1)
        #cv2.putText(frame, str(center), (10, 100), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
        cv2.putText(frame, str(w), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)

    # update the points queue
    #pts.appendleft(center)
    # show the frame to our screen
    cv2.line(frame, (313, 0), (313, 480), (255, 0, 0), 1)
    cv2.imshow("Frame", frame  )
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()