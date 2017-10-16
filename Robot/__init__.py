import driveTest as drive
class Robot:
    turningLeft = False
    turningRight = False
    #ballMiddle = False
    #basketMiddle = False
    drivingStraight = False

    def driveStraight(self):
        if self.drivingStraight:
            return
        drive.setspeed(90)
        self.drivingStraight = True
        self.turningRight = False
        self.turningLeft = False
    def turnLeft(self):
        if self.turningLeft:
            return
        drive.spinleft()
        self.turningLeft = True
        self.turningRight = False
        self.drivingStraight = False
    def turnRight(self):
        if self.turningRight:
            return
        drive.spinright()
        self.turningRight = True
        self.turningLeft = False
        self.drivingStraight = False
    def stopMoving(self):
        drive.shutdown()
        self.turningRight = False
        self.turningLeft = False
        self.drivingStraight = False
    def turnToBall(self, ballx, dif):
        if ballx < 320 - dif/2:
            self.turnRight()
        else:
            self.turnLeft()

class StageHandler:
    def __init__(self, robot):
        self.robot = robot