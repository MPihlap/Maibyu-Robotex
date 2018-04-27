# import the necessary packages
import numpy as np
import cv2
from MainboardComms import *
#blueLower = (24, 81, 41)
#blueUpper = (110, 255, 96)
blueLower = (30, 90, 52)
blueUpper = (47, 139, 76)

camera = cv2.VideoCapture(0)
def find_marker(image):
    # convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    # compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)
knownWidth = 16
focallength = (48*192)/knownWidth
kernel = np.ones((2,2), np.uint8)
drive = DriveTest()
alusta = False
f = open("andmed.txt","w")
i = 0
f.write("i,kaugus,laius\n")
while True:

    if alusta:
        drive.setspeed(90, 10)
        i += 1

    (grabbed, frame) = camera.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, blueLower, blueUpper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    # convert the image to grayscale, blur it, and detect edges
    #opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    res = cv2.bitwise_and(hsv, hsv, mask=mask)  # uhendan hsv pildi maskitud pildiga
    opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)
    cv2.imshow("edged",edged)

    #marker = find_marker(res)
    #print(marker[1][0])
    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    #cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (5, 5), 0)
    #edged = cv2.Canny(gray, 35, 125)

    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    cnts = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    #c = max(cnts, key=cv2.contourArea)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 75:
            marker = cv2.minAreaRect(c)
            #print (focallength)
            #print(marker[0][1])
            #print((knownWidth*focallength)/marker[1][0])
        #x, y, w, h = find_marker(hsv)
        x, y, w, h = cv2.boundingRect(c)
        if w > 5:
            kaugus = knownWidth*focallength/w
            cv2.rectangle(frame,(x,y),(x+w,y+h),[0, 255, 255],2)
            cv2.putText(frame, "Laius: " + str(w), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
            cv2.putText(frame, "Kaugus: " + str(kaugus), (10, 350), cv2.FONT_HERSHEY_DUPLEX, 1,
                        cv2.COLOR_YUV420sp2GRAY)
            if i%10 == 0:
                f.write(str(i)+","+str(kaugus)+","+str(w)+"\n")
    #cv2.imshow("gray",gray)
    cv2.line(frame, (313, 0), (313, 480), (255, 0, 0), 1)
    cv2.imshow("frame",frame)
    #cv2.imshow("mask",mask)
    #print()
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        drive.shutdown()
        f.close()
        break
    if key == ord("p"):
        alusta = not alusta

