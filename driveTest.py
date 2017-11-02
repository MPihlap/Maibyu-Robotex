import serial
import math
import threading
import time


class DriveTest:
    def commandThread(self):
        while self.running:
            time.sleep(0.1)
            print(self.speed1)
            text = ("sd:" + str(self.speed1) + ":" + str(self.speed2) + ":" + str(self.speed3) + "\r\n")
            self.ser.write('f0\r\n'.encode('utf-8'))
            time.sleep(0.01)
            self.ser.write(text.encode('utf-8'))
            time.sleep(0.01)
            self.ser.write(('d:'+ str(self.throwSpeed)+ '\r\n').encode('utf-8'))


    def __init__(self):
        self.running = True
        self.w = threading.Thread(name='commandThread', target=self.commandThread)
        self.speed1 = 0
        self.speed2 = 0
        self.speed3 = 0
        self.dist = 0.115
        self.throwSpeed = 1000
        self.wheelone = 0
        self.wheeltwo = 120
        self.wheelthree = 240
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.01, dsrdtr=True)
        self.w.start()

    def stopThrow(self):
        self.throwSpeed= 1000

    def shutdown(self):
        self.speed1 = 0
        self.speed2 = 0
        self.speed3 = 0


    def spinright(self):

        self.speed1 = -9
        self.speed2 = -9
        self.speed3 = -9


    def spinleft(self):
        self.speed1 = 9
        self.speed2 = 9
        self.speed3 = 9


    def circleBallLeft(self):
        self.speed1 = 9
        self.speed2 = 0
        self.speed3 = 0

    def circleBallRight(self):
        self.speed1 = 0
        self.speed2 = 9
        self.speed3 = 0



    # print(self.ser.isOpen(self))

    # wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + wheelDistanceFromCenter * robotAngularVelocity

    def wheelLogic(self,robotspeed, wheel, suund):
        speed = robotspeed * math.cos(math.radians(suund - wheel))  # +dist*angVel
        return speed

    def startThrow(self):
        self.throwSpeed = 1600


    def setspeed(self,suund, speed):
        self.speed1 = int(self.wheelLogic(-speed, self.wheelone, suund))
        self.speed2 = int(self.wheelLogic(-speed, self.wheeltwo,suund))
        self.speed3 = int(self.wheelLogic(-speed, self.wheelthree, suund))


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
