#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2 as pc2 
from sensor_msgs.msg import LaserScan 
from laser_geometry import LaserProjection

class Laser2PointCloud():
    def __init__(self):
        self.laserProjection = LaserProjection()
        self.laserPub = rospy.Publisher("/laser/point_cloud",pc2, queue_size=1)
        self.subscriber = rospy.Subscriber("/laser/pointer", LaserScan, self.laserCallBack)

    def laserCallBack(self,data):
        cloud_out = self.laserProjection.projectLaser(data)
        self.laserPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PointCloud()
    rospy.spin()