import cv2
import serial
import numpy as np
frame = np.zeros((200,200))
ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.01, dsrdtr=True)
while(True):
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    print(key)
    #print(key)
    # if the 'q' key is pressed, stop the loop
    if key == ord("w"):
        ser.write("d:1000\r\n".encode("utf-8"))
    if key == ord("a"):
        ser.write("f::0\r\n".encode("utf-8"))
        ser.write("d:1600\r\n".encode("utf-8"))
    if key == ord("s"):
        ser.write("d:2000\r\n".encode("utf-8"))

    if key == ord("q"):
        ser.write("f::1\r\n".encode("utf-8"))
        ser.write("d:0\r\n".encode("utf-8"))
        ##cv2.imwrite("test.png", frame)
        break