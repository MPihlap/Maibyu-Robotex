import numpy as np
def readThrowStr(filename):
    f = open(filename)
    end = []
    for i in f:
        pieces = i.strip().split(",")
        dist = round(float(pieces[0]))
        str = round(float(pieces[1]))
        end.append((dist, str))
    return end

def wasdControl(key, drive):
    if key == ord("y"):
        drive.spinleft()
    if key == ord("u"):
        drive.spinright()
    if key == ord("w"):
        drive.setspeed(90, 40)
    if key == ord("a"):
        drive.setspeed(180, 40)
    if key == ord("s"):
        drive.setspeed(270, 40)
    if key == ord("d"):
        drive.setspeed(0, 40)
    if key == 32:  # spacebar to stop
        drive.shutdown()

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
