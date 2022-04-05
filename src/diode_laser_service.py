#!/usr/bin/env python

import rospy
from robot_cognition_system.srv import DiodeLaserService, DiodeLaserServiceResponse
import serial
import time

ser = serial.Serial("/dev/ttyACM0",9600)

class DiodeLaserServiceNode():
    def __init__(self):
        self.diode_service = rospy.Service("diode_laser_service", DiodeLaserService, self.service_handler)

    def service_handler(self,data):
        print(data.service_command_name)
        print(data.service_command_value)
        cmd_value = data.service_command_value
        if(data.service_command_name == "pan"):
            cmd_value = int(data.service_command_value) + 80

        if(data.service_command_name == "tilt"):
            cmd_value = int(data.service_command_value) * 2.20

        print(cmd_value)
        command = "{name}_{value}".format(name=data.service_command_name, value=cmd_value)
        print(command)
        ser.write(command)
        time.sleep(1)
        return DiodeLaserServiceResponse(True)

if __name__ == '__main__':
    rospy.init_node("diode_laser_service")
    node_service = DiodeLaserServiceNode()
    rospy.spin()