from driveTest import *
frame = np.zeros((200,200))
while(True):
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    #print(key)
    # if the 'q' key is pressed, stop the loop
    if key == ord("w"):
        setspeed(90)
    if key == ord("a"):
        setspeed(180)
    if key == ord("s"):
        setspeed(270)
    if key == ord("d"):
        setspeed(0)
    if key == ord(" "):
        shutdown()
    if key == ord("r"):
        circleBall()

    if key == ord("q"):
        shutdown()
        ##cv2.imwrite("test.png", frame)
        break