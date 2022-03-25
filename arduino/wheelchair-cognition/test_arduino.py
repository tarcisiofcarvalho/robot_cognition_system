import serial

ser = serial.Serial("/dev/ttyACM1",9600)

while 1:
    var=raw_input("Enter 0 or 1: ")
    ser.write(var)

# pan = 0 - 80 - 160
# tilt = 1 - 80


# define the math conversion from deg/rad to servo motor 0>255
# code arduino to receive <servo>:<deg/rad> and apply in servo
# code ROS Node to receive <servo>:<deg/rad> and send to arduino