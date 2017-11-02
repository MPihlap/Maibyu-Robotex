from collections import deque
import cv2


# define the lower and upper boundaries of the
# ball in the HSV color space, then initialize the
# list of tracked points
#greenLower = (49, 35, 72)
#greenUpper = (80, 108, 204)
#greenLower = (0, 113, 126)
#greenUpper = (13, 196, 255)
greenLower = (5,255,43) #test roheline vaartus
greenUpper= (38,255,160)
blueLower = (26,89,52)  #
blueUpper = (179,185,114)
#blueLower = (101,107,94)
#blueUpper = (115,147,134)
pts = deque()

#grab the reference to the webcam
camera = cv2.VideoCapture(0)

while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    res = cv2.bitwise_and(hsv, hsv, mask=mask)
    cv2.imshow("res",res)
    cv2.imshow("mask",mask)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 0:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points

            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, str(center), (10, 100), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
            cv2.putText(frame, str(round(radius*2,0)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)

    # update the points queue
    pts.appendleft(center)

    # show the frame to our screen
    cv2.imshow("Frame", frame  )
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()