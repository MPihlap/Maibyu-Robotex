import serial
import math
import threading
import time


class DriveTest:
    def commandThread(self):
        while self.running:
            # time.sleep(0.05)
            # print(self.speed1)
            vastus = self.ser.read(19)
            if len(vastus) > 0:
                self.rfcommand += vastus
                print("vahepealne: "+vastus)
                if len(self.rfcommand) == 19:
                    #Hi there
                    self.gameOn = self.parseRefCommand(self.rfcommand,self.field,self.robotChar)
                    print(self.gameOn)
                    self.rfcommand = ""
            text = ("sd:" + str(self.speed1) + ":" + str(self.speed2) + ":" + str(self.speed3) + "\r\n")
            #self.ser.write('f0\r\n'.encode('utf-8'))
            #time.sleep(0.002)
            self.ser.write(text.encode('utf-8'))
            time.sleep(0.002)
            self.ser.write(('d:'+ str(self.throwSpeed)+ '\r\n').encode('utf-8'))


    def __init__(self):
        self.rfcommand = ""
        self.running = True
        self.w = threading.Thread(name='commandThread', target=self.commandThread)
        self.speed1 = 0
        self.speed2 = 0
        self.speed3 = 0
        self.dist = 0.115
        self.f = open("RFinfo.txt")
        self.field = self.f.readline().split("=")[1].split("#")[0].strip()
        self.robotChar = self.f.readline().split("=")[1].split("#")[0].strip()
        self.n = 'rf:a' + self.field + self.robotChar
        self.f.close()
        self.throwSpeed = 1000
        self.wheelone = 0
        self.wheeltwo = 120
        self.wheelthree = 240
        self.gameOn = False
        self.ser = serial.Serial('COM3', baudrate=9600, timeout=0.005)
        self.lastSpeed = 0
        self.w.start()

    def stopThrow(self):
        self.throwSpeed = 1000

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

    def circleBallLeft(self, speed):
        self.speed1 = speed  # 9
        self.speed2 = 0
        self.speed3 = 0

    def circleBallRight(self, speed):
        self.speed1 = -speed
        self.speed2 = 0
        self.speed3 = 0

    def parseRefCommand(self, command, field, robotChar):
        print("cmd " + command)
        pieces = command.split(":")
        if pieces[1][1] != field:
            return self.gameOn
        if pieces[1][2] != robotChar and pieces[1][2] != "X":
            return self.gameOn
        if "START" in command:
            self.ser.write(self.n + 'ACK-----\r\n')
            return True
        elif "STOP" in command:
            self.ser.write(self.n + 'ACK-----\r\n')
            self.shutdown()
            return False
        elif "PING" in command:
            self.ser.write(self.n + 'ACK-----\r\n')
            return self.gameOn

    # print(self.ser.isOpen(self))

    # wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + wheelDistanceFromCenter * robotAngularVelocity

    def wheelLogic(self, robotspeed, wheel, suund):
        speed = robotspeed * math.cos(math.radians(suund - wheel))  # +dist*angVel
        return speed

    def startThrow(self, speed):
        self.throwSpeed = speed

    def setspeed(self, suund, speed):
        if speed - self.lastSpeed > 50:
            newSpeed = self.lastSpeed + 2
            self.lastSpeed = newSpeed
            speed = newSpeed
        self.speed1 = int(self.wheelLogic(-speed, self.wheelone, suund))
        self.speed2 = int(self.wheelLogic(-speed, self.wheeltwo, suund))
        self.speed3 = int(self.wheelLogic(-speed, self.wheelthree, suund))