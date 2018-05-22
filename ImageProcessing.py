import cv2
from UtilityFunctions import readin
import numpy as np

MIN_BASKET_AREA = 400
MIN_BALL_AREA = 30
greenLower, greenUpper = readin("Pall.txt")
##borderLower, borderUpper = readin("mustpiir.txt")
teamPink = True
basketLower = None
basketUpper = None
kernelBasket = np.ones((2, 2), np.uint8)
kernel = np.ones((2, 2), np.uint8)

def setOpposingBasketThresh(teamPink):
    global basketLower, basketUpper
    if teamPink:  # Attacking blue basket
        basketLower, basketUpper = readin("VaravSinineB.txt")
    else:  # Attacking pink basket
        basketLower, basketUpper = readin("VaravLillaB.txt")

def setTeam(team):
    global teamPink
    teamPink = team == "pink"
    setOpposingBasketThresh(teamPink)

def drawThing(frame, cnts, isBall):
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    pindala = cv2.contourArea(c)

    if isBall:
        if pindala < MIN_BALL_AREA:
            return -1, -1
    else:
        if pindala < MIN_BASKET_AREA:
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

def contourify(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    maskBlue = cv2.inRange(hsv, basketLower, basketUpper)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_CLOSE, kernelBasket)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_OPEN, kernelBasket)
    maskCombo = cv2.add(mask, maskBlue)
    cv2.imshow("combo", maskCombo)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cntsPurple = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[-2]

    return cnts, cntsPurple