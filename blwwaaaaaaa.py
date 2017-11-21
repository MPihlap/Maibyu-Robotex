import serial

ser = serial.Serial('COM3',baudrate=115200,timeout=0.1)
liikumine = False
n = 'rf:aAA'
ser.flush()
while True:
    vastus = ser.read(19)
    #vastus = ser.readline()
    #if len(vastus) > 0:
     #   print(vastus)
    vastus2= str(vastus)
    ser.flush()
    #print("a")
    if len(vastus2) > 0:
        print(vastus2)
    if vastus2 == '<ref:aAASTART---->\n':
        ser.write('rf:aAAACK-----\n')
        print("sain start kasu")
        liikumine = True
        #ser.flush()
    elif vastus2 == '<ref:aAASTOP----->\n':
        ser.write('rf:aAAACK-----\n')
        print("sain stop kasu")
        liikumine = \
            False
    elif vastus2 == '<ref:aAAPING----->\n':
        print("sain ping kasu")
        ser.write(n+'ACK-----\n')
