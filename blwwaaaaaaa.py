import serial

ser = serial.Serial('COM4',baudrate=115200,timeout=0.01)
liikumine = False
n = 'rf:aAB'
while True:
    vastus = ser.read(19)
    vastus2= str(vastus,'UTF-8')
    print(vastus)
    print(vastus2)
    print(ser.inWaiting())
    if vastus == 'aAXSTART----':
        ser.write(n + 'ACK-----')
        print("sain start kasu")
        liikumine = True
    elif vastus == 'aAXSTOP------':
        ser.write(n + 'ACK-----')
        print("sain stop kasu")
        liikumine = False
    elif vastus == n + 'PING-----':
        print("sain ping kasu")
        ser.write(n + 'ACK-----')
