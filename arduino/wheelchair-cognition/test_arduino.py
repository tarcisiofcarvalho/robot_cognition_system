import serial

ser = serial.Serial("/dev/ttyACM0",9600)

while 1:
    var=raw_input("Enter 1 or 0")
    ser.write(var)