import serial

ser = serial.Serial("/dev/ttyACM0",9600)

while 1:
    var=raw_input("Enter 0 or 1: ")
    ser.write(var)

# pan = 0 - 80 - 160
# tilt = 3 - 80


# define the math conversion from deg/rad to servo motor 0>255
# code ROS Node to receive <servo>:<deg/rad> and send to arduino