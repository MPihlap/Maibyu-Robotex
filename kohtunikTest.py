import serial


ser = serial.Serial('COM3',baudrate=115200,timeout = 0.8,dsrdtr=True)
liikumine = False
n = 'aAB'
while True:
    vastus = ser.read(12);
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
