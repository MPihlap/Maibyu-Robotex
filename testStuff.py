from collections import deque
import cv2

def trackObject(isBall, camera, pts):
    if isBall:
        #Ball colour values
        lower = (49, 35, 72)
        upper = (80, 108, 204)
    else:
        #Basket colour values
        #lower = (101, 107, 94)
        #upper = (115, 147, 134)
        lower = (90, 90, 90)
        upper = (120, 150, 140)

    while True:
        # grab the current frame
        (grabbed, frame) = camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
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
            if isBall:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 5:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points

                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    cv2.putText(frame, str(center), (10, 100), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
                    cv2.putText(frame, str(round(radius * 2, 0)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                                cv2.COLOR_YUV420sp2GRAY)
            else:
                x,y,w,h = cv2.boundingRect(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if x > 10 and y > 10:
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0, 255, 255), 2)
                    cv2.putText(frame, "Laius: "+str(round(w)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                            cv2.COLOR_YUV420sp2GRAY)
                cv2.putText(frame, "Pikkus: " + str(round(h)), (10, 350), cv2.FONT_HERSHEY_DUPLEX, 1,
                            cv2.COLOR_YUV420sp2GRAY)



        # update the points queue
        pts.appendleft(center)

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
pts = deque()

# grab the reference to the webcam
camera = cv2.VideoCapture(0)
trackObject(False, camera, pts)
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
