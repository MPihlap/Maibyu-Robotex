from driveTest import DriveTest
from collections import deque
import cv2
import time
import numpy as np
distBuffer = deque()
drive = DriveTest()
keskX = 317
f = open("visketugevused.csv","a")
def throwStr(distance):
    return 1165 + distance #1150 lhedalt
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
            # print("Raadius: "+ str(radius))
            # only proceed if the radius meets a minimum size
            if radius > 3:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points

                cv2.circle(image, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image, center, 5, (0, 0, 255), -1)
                cv2.putText(image, str(center), center, cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
                cv2.putText(image, str(round((radius ** 2) * 3.14)), (center[0] + 200, center[1]),
                            cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
        return x, y
    else:
        #x, y, w, h = cv2.boundingRect(c)

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

        # qprint (x,y,w,h)
        if w > 5:
            #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
            cv2.putText(image, "Laius: " + str(round(w)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
        return x, y, w, h
        # return 0, 0, 0, 0
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
teamPink = True
if teamPink:  # Attacking blue basket
    basketLower, basketUpper = readin("VaravsinineB.txt")
    #basketLower = (100, 79, 124)
    #basketUpper = (108, 145, 191)
else:  # Attacking pink basket
    basketLower, basketUpper = readin("VaravLillaB.txt")

camera = cv2.VideoCapture(0)
def none(x):
    pass
counter = 0
cv2.namedWindow("image")
cv2.createTrackbar("Hundreds",'image',0,5,none)
cv2.createTrackbar("Tens",'image',0,9,none)
cv2.createTrackbar("Ones",'image',0,9,none)
kernelBasket = np.ones((4, 4), np.uint8)
time.sleep(1)
drive.stopThrow()

knownWidth = 16
#focallength = (64 * 128) / knownWidth

focallength = (59 * 151)/ knownWidth
while True:
    ret, image = camera.read()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    maskBlue = cv2.inRange(hsv, basketLower, basketUpper)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_CLOSE, kernelBasket)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_OPEN, kernelBasket)
    cntsPurple = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[-2]
    distance = 0

    if len(cntsPurple) > 0:
        basketx, baskety, w, h = drawThing(cntsPurple, False)
        # lisatav number on korvi laius(vaja kontrollida, kas on ikka 8)
        distance = 8 + (knownWidth * focallength / w)
        cv2.putText(image, "Kaugus: " + str(distance), (10, 200), cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)
        cv2.putText(image, "Pindala: " + str(cv2.contourArea(max(cntsPurple, key=cv2.contourArea))), (10, 250),
                    cv2.FONT_HERSHEY_DUPLEX, 1,
                    cv2.COLOR_YUV420sp2GRAY)

    hundreds = cv2.getTrackbarPos("Hundreds","image")*100
    tens = cv2.getTrackbarPos("Tens","image")*10
    ones = cv2.getTrackbarPos("Ones","image")
    speed = 1000+hundreds+tens+ones
    #cv2.putText(image, "kiirus: " + str(speed), (20, 240), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV2GRAY_420)
    cv2.line(image, (keskX, 0), (keskX, 480), (255, 0, 0), 1)
    cv2.imshow("image", image)
    counter+= 1
    #speed = 1000
    #if distance > 0:
    #    speed = throwStr(distance)
    if distance > 0:
        distBuffer.append(distance)

    """if len(distBuffer) > 0:
        mindist = min(distBuffer)
        print mindist
        if mindist > 0:
            speed = throwStr(mindist)
    """
    if len(distBuffer) > 10:
        distBuffer.popleft()
    if counter == 5:
        counter = 0
        drive.startThrow(speed)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("m"):
        #print str(min(distBuffer)) + "," +str(speed)+"\n"
        print str(max(distBuffer)) + "," +str(speed)+"\n"
        #f.write(str(round(min(distBuffer), 2)) + "," +str(speed)+"\n")
        f.write(str(round(max(distBuffer), 2)) + "," +str(speed)+"\n")
    if key == ord("q"):
        ##cv2.imwrite("test.png", frame)
        drive.shutdown()
        drive.stopThrow()
        break
# cleanup the camera and close any open windows
drive.running = False
camera.release()
f.close()
cv2.destroyAllWindows()
print "tsuklist valjas"
