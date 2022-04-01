#!/usr/bin/env python

import sys
import rospy
from robot_cognition_system.srv import DiodeLaserService, DiodeLaserServiceResponse
import time

def test(cmd_name, cmd_value):
    rospy.wait_for_service('diode_laser_service')
    try:
        test_service = rospy.ServiceProxy('diode_laser_service', DiodeLaserService)
        resp1 = test_service(cmd_name, cmd_value)
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    test('pan','60')
    test('tilt','10')
    test('led','on')