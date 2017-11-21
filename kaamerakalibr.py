import cv2

camera = cv2.VideoCapture(0)

"""11. CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
12. CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
13. CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
14. CV_CAP_PROP_HUE Hue of the image (only for cameras).
15. CV_CAP_PROP_GAIN Gain of the image (only for cameras).
16. CV_CAP_PROP_EXPOSURE Exposure (only for cameras)."""

def nothing(x):
    pass


cv2.namedWindow('image')
cv2.createTrackbar('Brightness','image',0,300,nothing)         #loob trackbarid pildist varvide eraldamiseks
cv2.createTrackbar('Contrast','image',0,300,nothing)
cv2.createTrackbar('Saturation','image',0,300,nothing)
cv2.createTrackbar('Hue','image',0,300,nothing)
cv2.createTrackbar('Gain','image',0,300,nothing)
cv2.createTrackbar('Exposure','image',0,300,nothing)
cv2.createTrackbar('White-balance','image',0,1,nothing)
while True:
    # take frame
    ret, image = camera.read()
    # BGR to HSV
    Brightness = cv2.getTrackbarPos('Brightness', 'image')/50-3
    print(cv2.CAP_PROP_SATURATION)
    Contrast = cv2.getTrackbarPos('Contrast', 'image')
    Saturation = cv2.getTrackbarPos('Saturation', 'image')
    Hue = cv2.getTrackbarPos('Hue', 'image')
    Gain = cv2.getTrackbarPos('Gain', 'image')
    Exposure = cv2.getTrackbarPos('Exposure', 'image')
    WB = cv2.getTrackbarPos('White-balance','image')
    cv2.imshow("image",image)
    camera.set(5,Brightness)
    camera.set(11, Contrast)
    camera.set(cv2.CAP_PROP_SATURATION, Saturation)
    camera.set(cv2.CAP_PROP_HUE, Hue)
    camera.set(14, Gain)
    camera.set(cv2.CAP_PROP_EXPOSURE, Exposure)
    camera.set(18,WB)
    k = cv2.waitKey(1)
    if k == 27:
        # salvestan praegused h1-v2 muutujad, et need kirjutada faili
        print('olen siin')
        break
camera.release()
cv2.destroyAllWindows()