#vajab veel natuke tuunimist, igaks juhuks varvid veel uhe korra ule vaadata, eelisjarjekorras pall

#aared praegu ainult valged
import numpy as np
import cv2
print(cv2.__version__)

cap = cv2.VideoCapture(0)

img = np.zeros((480, 640, 3), np.uint8) # tuhi must pilt , selle peale joonistatakse kontuurid
kernel = np.ones((2,2), np.uint8)       # tuhi valge pilt, kasutatakse opening ja closing juures
#opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)  eemaldab valised tapid
#closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) eemaldab sisemised tapid

def nothing(x):
    pass


def rememberpos(Lst,fail): # salvestab programmi sulgedes praegused threshholdid
    pos = open(fail, "w")
    for i in range(len(Lst)):
        if i == len(Lst) - 1:
            pos.write(str(Lst[i]))
        else:
            pos.write(str(Lst[i]) + ",")
    pos.close()


def readin(filename):
    read = open(filename, "r")
    f = read.readlines()
    algne = f[0].split(",")
    if len(algne) == 0 :
        return [0,0,0], [179,255,255]
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


cap.set(18,False)

cv2.namedWindow('image')
cv2.createTrackbar('h1','image',0,179,nothing)         #loob trackbarid pildist varvide eraldamiseks
cv2.createTrackbar('h2','image',179,179,nothing)
cv2.createTrackbar('s1','image',0,255,nothing)
cv2.createTrackbar('s2','image',255,255,nothing)
cv2.createTrackbar('v1','image',0,255,nothing)
cv2.createTrackbar('v2','image',255,255,nothing)

#def varvithresholdid(filename): # loeb failist sisse varvi thresholdid
#    if len(readin(filename)[0].split(",")) != 0:
#        for i in range(0,5):
#            cv2.setTrackbarPos(varnames[i],"image",int(readin(filename)[0].split(",")[i]))

varnames = ["h1","h2","s1","s2","v1","v2"]

#lower, upper = readin("Pall.txt") # loeb enne tsukli algust sisse mingid thresholdid
#print(lower)
#print(upper)
#lower = np.array(lower)
#upper = np.array(upper)
#lower,upper = readin("VaravKollane.txt")

while True:
    #take frame
    ret, frame = cap.read()
    #BGR to HSV
    cv2.imshow("pilt", frame)
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv2', hsv2)

    h1 = cv2.getTrackbarPos('h1', 'image')
    h2 = cv2.getTrackbarPos('h2', 'image')
    s1 = cv2.getTrackbarPos('s1', 'image')
    s2 = cv2.getTrackbarPos('s2', 'image')
    v1 = cv2.getTrackbarPos('v1', 'image')
    v2 = cv2.getTrackbarPos('v2', 'image')

    #limits
    #lower = np.array(lower)
    #upper = np.array(upper)
    lower = np.array([h1,s1,v1])
    upper = np.array([h2,s2,v2])
    #Threshold the image
    mask = cv2.inRange(hsv,lower,upper)
    cv2.imshow('image', mask)

    res = cv2.bitwise_and(hsv, hsv, mask=mask)           #uhendan hsv pildi maskitud pildiga
    opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)  # tootlen pilti, et eemaldada ebavajalikud osad
    cv2.imshow('opening', opening)
    # suurima kontuuri leidmine
    pilt = cv2.cvtColor(opening, cv2.COLOR_BGR2GRAY)  # findcontours nouab halliks tehtud pilti
    im2, contours, hierarchy = cv2.findContours(pilt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    suurim = 10
    suurimindeks = -1

    # muutujad jargneva for tsukli jaoks
    for i in range(0, len(contours)):  # leiab suurima kontuuri
        x = cv2.contourArea(contours[i])
        if x > suurim:
            suurim = x
            suurimindeks = i

    img = np.zeros((480, 640, 3), np.uint8) #loob musta ekraani
    mustkast = np.zeros((480, 640, 3), np.uint8)
    if suurimindeks == -1:
        kontuursuurim = mustkast
    else:
        print(suurim)
        kontuursuurim = cv2.drawContours(img, contours, suurimindeks, (0, 255, 0), 3) #joonistab suurima kontuuri mustale ekraanie
    #if suurim > 10 :            # palli tuvastus test
    #    print("ball")
    #else:
    #    print("no ball")
    kontuursuurim = cv2.cvtColor(kontuursuurim, cv2.COLOR_BGR2GRAY)
    M = cv2.moments(kontuursuurim)
    if M["m00"] == 0:
        kontuursuurim = mustkast
    else:
        mustkast = np.zeros((480, 640, 1), np.uint8)
        cX = int(M["m10"] / M["m00"])
        print(cX)
        cY = int(M["m01"] / M["m00"])
        kontuursuurim = cv2.bitwise_or(cv2.circle(mustkast, (cX, cY), 1, (255, 0, 0), -1), kontuursuurim)
    cv2.imshow("kontuurid", kontuursuurim)
    k = cv2.waitKey(1)
    print(k)
    if k == 27:
        L = [h1, h2, s1, s2, v1, v2] # salvestan praegused h1-v2 muutujad, et need kirjutada faili
        print('olen siin')
        rememberpos(L, "VaravLilla.txt")
        break
cv2.destroyAllWindows()