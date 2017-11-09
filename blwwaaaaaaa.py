import serial

ser = serial.Serial('/dev/ttyACM0',baudrate=9600,timeout=0.1)
liikumine = False
n = 'rf:aAB'
while True:
    vastus = ser.read(19)
    #if len(vastus) > 0:
     #   print(vastus)
    vastus2= str(vastus)
    ser.flush()
    #print("a")
    if len(vastus2) > 0:
        print(vastus2)
    if vastus2 == '<ref:aKLSTART---->\n':
        ser.write(n + 'ACK-----')
        print("sain start kasu")
        liikumine = True
    elif vastus2 == '<ref:aAXSTOP----->\n':
        ser.write(n + 'ACK-----')
        print("sain stop kasu")
        liikumine = False
    elif vastus2 == '<ref:aKLPING----->':
        print("sain ping kasu")
        ser.write(n + 'ACK-----')
