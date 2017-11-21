from driveTest import DriveTest
import cv2
import time

drive = DriveTest()
camera = cv2.VideoCapture(0)
def none(x):
    pass
counter = 0
cv2.namedWindow("image")
cv2.createTrackbar("Hundreds",'image',0,5,none)
cv2.createTrackbar("Tens",'image',0,9,none)
cv2.createTrackbar("Ones",'image',0,9,none)
time.sleep(1)
drive.stopThrow()


while True:
    ret, image = camera.read()
    hundreds = cv2.getTrackbarPos("Hundreds","image")*100
    tens = cv2.getTrackbarPos("Tens","image")*10
    ones = cv2.getTrackbarPos("Ones","image")
    speed = 1000+hundreds+tens+ones
    cv2.putText(image, "kiirus: " + str(speed), (20, 240), cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV2GRAY_420)
    cv2.imshow("image",image)
    counter+= 1

    if counter == 5:
        counter = 0
        drive.startThrow(speed)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        ##cv2.imwrite("test.png", frame)
        drive.shutdown()
        drive.stopThrow()
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
