from collections import deque
import cv2
import numpy as np
from MainboardComms import DriveTest


class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    # Template method:
    def runAll(self, inputs):
        pass

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print ('Processing current state:', str(self))

    def on_event(self, camera, frame):
        """
        Handle events that are delegated to this State.
        """
        pass


    def contourify(self, frame):
        #is bad
        global borderLower, borderUpper, kernel, greenLower,greenUpper,basketLower,basketUpper,kernelBasket
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        maskBlack = cv2.inRange(hsv, borderLower, borderUpper)
        maskBlack = cv2.morphologyEx(maskBlack, cv2.MORPH_OPEN, kernel)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # mask = cv2.erode(mask, None, iterations=1)
        # mask = cv2.dilate(mask, None, iterations=1)
        maskBlue = cv2.inRange(hsv, basketLower, basketUpper)
        maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_CLOSE, kernelBasket)
        maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_OPEN, kernelBasket)
        maskCombo = cv2.add(mask, maskBlue)
        cv2.imshow("combo", maskCombo)
        # maskBlue = cv2.erode(maskBlue, None, iterations=1)
        # maskBlue = cv2.dilate(maskBlue, None, iterations=1)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        cntsPurple = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)[-2]

        return cnts, cntsPurple

    def throwStrength(distance):
        distance = distance - 10
        for pair in throwStrengths:
            if distance == pair[0]:
                print("found equal distance")
                return pair[1]
            if distance < pair[0] and throwStrengths.index(pair) != 0:
                previous = throwStrengths[throwStrengths.index(pair) - 1]
                return previous[1] + (pair[1] - previous[1]) / 2
            elif throwStrengths.index(pair) == 0 and distance < pair[0]:
                return throwStrengths[0][1]
        return 1600

    def drawThing(self,cnts, isBall, frame):
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        pindala = cv2.contourArea(c)

        if isBall:
            if pindala < 30:
                return -1, -1
        else:
            if pindala < 400:
                return -1, -1, -1, -1
        if isBall:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # print("Raadius: " + str(radius))
                # only proceed if the radius meets a minimum size
                if radius > 3:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points

                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    cv2.putText(frame, str(center), center, cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
                    cv2.putText(frame, str(round((radius ** 2) * 3.14)), (center[0] + 200, center[1]),
                                cv2.FONT_HERSHEY_DUPLEX, 1, cv2.COLOR_YUV420sp2GRAY)
            return x, y
        else:
            # x, y, w, h = cv2.boundingRect(c)

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # if w > 5:
            rotation = rect[2]
            if rotation < -10:
                h = rect[1][0]
                w = rect[1][1]
            else:
                h = rect[1][1]
                w = rect[1][0]
            x = rect[0][0]
            y = rect[0][1]

            # qprint (x,y,w,h)
            if w > 5:
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                cv2.putText(frame, "Laius: " + str(round(w)), (10, 300), cv2.FONT_HERSHEY_DUPLEX, 1,
                            cv2.COLOR_YUV420sp2GRAY)
            return x, y, w, h
            # return 0, 0, 0, 0

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__

class SimpleDevice(object):
    """
    A simple state machine that mimics the functionality of a device from a
    high level.
    """

    def __init__(self):
        """ Initialize the components. """
        camera = cv2.VideoCapture(0)

        # Start with a default state.
        self.state = findBall()

    def on_event(self, event):
        """
        This is the bread and butter of the state machine. Incoming events are
        delegated to the given states which then handle the event. The result is
        then assigned as the new state.
        """

        # The next state will be the result of the on_event function.
        self.state = self.state.on_event(event)

class findBall(State):
    """
    no ball visible, twirl
    """

    def run(self):
        (grabbed, frame)= camera.read()
        cnts, cntsPurple = State.contourify(frame)
        ballX,ballY = self.drawThing(cnts,frame,True)
        return self
# End of our states.
class moveToBall(State):
    """
    ball visible, move to it
    """

    def on_event(self,camera,frame):
        ballX,ballY = self.drawThing(frame,True)

        return self
# End of our states.

class circleBall(State):
    """
    circle the ball
    """

    def on_event(self,camera,frame):
        ballX,ballY = self.drawThing(frame,True)
        return self
# End of our states.
class makeThrow(State):
    """
    ball and goal lined up, ready to throw.
    """

    def run(self,frame):
        basketballRobot.throwCounter += 1
        # basketx, baskety, w, h = drawThing(cntsPurple, False)
        drive.setspeed(90, 10)
        if basketballRobot.throwCounter == 1:
            if len(distBuffer) > 0:
                maxdist = max(distBuffer)
                # mindist = min(distBuffer)
                print(maxdist)
                # print(mindist)
                # if mindist > 0:
                #    speed = throwStrength(mindist)
                if maxdist > 0:
                    speed = State.throwStrength(maxdist)
                drive.startThrow(speed)
                print(speed)
        elif basketballRobot.throwCounter > 60:
            drive.stopThrow()
            distBuffer.clear()
            basketballRobot.throwCounter = 0
            nextState = "find"
    def next(self):
        if nextState == "find":
            return basketballRobot.findBall
        return basketballRobot.makeThrow


class basketballRobot(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,basketballRobot.findBall)



# static variables for basketballRobot
def readin(filename):
    read = open(filename, "r")
    f = read.readlines()
    algne = f[0].split(",")
    if len(algne) == 0:
        return [0, 0, 0], [179, 255, 255]
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

def readThrowStr(filename):
    f = open(filename)
    end = []
    for i in f:
        pieces = i.strip().split(",")

        dist = round(float(pieces[0]))
        str = round(float(pieces[1]))
        #if dist > 200:
        #s    str -= 10
        end.append((dist, str))
    return end

camera = cv2.VideoCapture(0)
drive =  DriveTest()
nextState = ""
gameIsOn = False
f = open("RFinfo.txt")
field = f.readline().split("=")[1].split("#")[0].strip()
robotChar = f.readline().split("=")[1].split("#")[0].strip()
n = 'rf:a' + field + robotChar
#296
knownWidth = 16
#knownWidth = 50
#8530
distBuffer = deque()
focallength = (59 * 151) / knownWidth
#focallength = (129 * 120) /knownWidth
throwStrengths = sorted(readThrowStr("visketugevused.csv"))
for i in throwStrengths:
    print(i)
greenLower, greenUpper = readin("Pall.txt")
borderLower, borderUpper = readin("mustpiir.txt")
teamPink = True
moveMid = False
startLimit = 75
if teamPink:  # Attacking blue basket
    basketLower, basketUpper = readin("VaravsinineB.txt")
else:  # Attacking pink basket
    basketLower, basketUpper = readin("VaravLillaB.txt")
basketIsMiddle = False
soidanOtse = False
circlingBall = False
makeThrow = False
keskX = 307
basketSmall = keskX - 7
basketLarge = keskX + 7
basketballRobot.throwCounter = 0
startCounter = 0
basketIsLeft = None
pts = deque()
img = np.zeros((480, 640, 3), np.uint8)
kernelBasket = np.ones((2, 2), np.uint8)
basketballRobot.findBall   = findBall()
basketballRobot.circleBall = circleBall()
basketballRobot.moveToBall = moveToBall()
basketballRobot.makeThrow  = makeThrow()

#variables end

