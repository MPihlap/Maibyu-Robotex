import serial
import math
import cv2
import numpy as np

dist = 0.115
wheelone = 0
wheeltwo = 120
wheelthree = 240

ser = serial.Serial('COM3', baudrate=115200, timeout=0.8, dsrdtr=True)


# print(ser.isOpen())

# wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + wheelDistanceFromCenter * robotAngularVelocity

def wheelLogic(robotspeed, wheel, dist, suund):
    angVel = robotspeed / 0.035
    speed = robotspeed * math.cos(math.radians(suund - wheel))  # +dist*angVel
    return speed


def shutdown():
    ser.write('sd0:0:0\r\n'.encode('utf-8'))


def spinright():
    ser.write('sd-9:-9:-9\r\n'.encode('utf-8'))


def spinleft():
    ser.write('sd9:9:9\r\n'.encode('utf-8'))


def circleBallLeft():
    ser.write('sd5:9:9\r\n'.encode('utf-8'))
def circleBallRight():
    ser.write('sd9:5:9\r\n'.encode('utf-8'))
def circleBallWat():
    ser.write('sd9:9:5\r\n'.encode('utf-8'))


def setspeed(suund):
    spd1 = int(wheelLogic(-20, wheelone, dist, suund))
    spd2 = int(wheelLogic(-20, wheeltwo, dist, suund))
    spd3 = int(wheelLogic(-20, wheelthree, dist, suund))
    text = ("sd" + str(spd1) + ":" + str(spd2) + ":" + str(spd3) + "\r\n")
    ser.write('f0\r\n'.encode('utf-8'))
    ser.write(text.encode('utf-8'))
    """"""


#    print("mootor1")
#   print(spd1)
# print("mootor2")
#  print(spd2)
# print("mootor3")
# print(spd3)
"""
setspeed(180)
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

    if key == ord("q"):
        shutdown()
        ##cv2.imwrite("test.png", frame)
        break

"""

# print(wheelLogic(1,wheelone,dist,1,180))
