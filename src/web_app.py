#!/usr/bin/env python

import rospy 
from std_msgs.msg import String, Float64
from flask import Flask, request, jsonify, render_template
import threading
import math
import move_to_target as move_action
import time
from sensor_msgs.msg import LaserScan 

passage_condition_result = "test"

app = Flask(__name__)

class PassageCondition():

    passage_condition = "test"
    pan_deg = 0
    tilt_deg = 0
    laser_range = 0
    target_distance = 0
    pub_laser_pan = None
    pub_laser_tilt = None
    pub_laser_pan_sim = None
    pub_laser_tilt_sim = None    

    def __init__(self):
        self.subscriber = rospy.Subscriber("/passage_condition", String, self.passage_classification)
        self.subscriber_laser = rospy.Subscriber("/laser/pointer", LaserScan, self.laser_data)
        self.subscriber_target_distance = rospy.Subscriber("/target_distance", Float64, self.target_distance_data)             
        self.passage_condition = "test"
        self.laser_pan_tilt_set()
        

    def passage_classification(self, data):
        PassageCondition.passage_condition = data
        

    def laser_data(self, data):
        PassageCondition.laser_range = data

    def target_distance_data(self, data):
        PassageCondition.target_distance = data.data        


    def laser_pan_tilt_set(self):
        rate = rospy.Rate(100) # 10000hz
        PassageCondition.pub_laser_pan = rospy.Publisher('/r2d2_laser_pan_controller/command', Float64, queue_size=10)
        PassageCondition.pub_laser_tilt = rospy.Publisher('/r2d2_laser_tilt_controller/command', Float64, queue_size=10) 
        PassageCondition.pub_laser_pan_sim = rospy.Publisher('/laser_sim_pan', Float64, queue_size=10)
        PassageCondition.pub_laser_tilt_sim = rospy.Publisher('/laser_sim_tilt', Float64, queue_size=10)          
        while not rospy.is_shutdown():
            # rospy.loginfo("Pan_rad: " + str(math.radians(self.pan_deg)))
            # rospy.loginfo("Tilt_rad: " + str(math.radians(self.tilt_deg)))
            PassageCondition.pub_laser_pan.publish(math.radians(self.pan_deg))
            PassageCondition.pub_laser_tilt.publish(math.radians(self.tilt_deg))
            PassageCondition.pub_laser_pan_sim.publish(math.radians(self.pan_deg))
            PassageCondition.pub_laser_tilt_sim.publish(math.radians(self.tilt_deg))            
            rate.sleep()

def laser_pan_tilt_distance_to_XY_2D_in_meters(pan=None, tilt=None, distance=None):
    """
        This component will transform the laser pan and tilt orientation values and the 
        distance from robot to the move target to a point XY in the 2D plane
        Parameters:
        - pan: angle in radians
        - tilt: angle in radians
        - range: in meters
    """
    # https://www.mathworks.com/matlabcentral/answers/427558-how-can-i-create-xyz-coordinant-from-pan-tilt-system-angles
    # function [x, y, z] = my_sph2cart(pan,tilt,range)
    #     x = range .* cosd(tilt) .* cosd(pan);
    #     y = range .* cosd(tilt) .* sind(pan);
    #     z = range .* sind(tilt);
    # end

    # print("Pan: {0}".format(pan))
    # print("Tilt: {0}".format(tilt))
    print("distance: {0}".format(distance))

    # Convert distance to Int
    distance = int(round(distance))

    if pan is not None and tilt is not None and distance is not None:
        try:
            x = distance * math.cos(tilt) * math.cos(pan)
            y = distance * math.cos(tilt) * math.sin(pan)
            print("x: {0}".format(x))
            print("y: {0}".format(y))
            return x, y
        except e:
            print(e)
            return None, None

    return None, None

@app.route('/passage_condition', methods=['GET'])
def passage_condition_route():
    passage_condition_result = PassageCondition.passage_condition.data
    print("Passage condition: " + str(passage_condition_result))
    print("Target distance: " + str(PassageCondition.target_distance))
    
    # Get the Target X and Y from Robot Base
    x, y = move_action.transform_laser_pan_tilt_to_XY_2D_in_meters(pan=math.radians(PassageCondition.pan_deg), 
                                                                   tilt=math.radians(PassageCondition.tilt_deg), 
                                                                   distance=PassageCondition.target_distance)

    return jsonify({'condition': str(passage_condition_result),
                    'target_distance': str(PassageCondition.target_distance),
                    'target_x': str(x),
                    'target_y': str(y)})


@app.route('/laser_move', methods=['POST'])
def pan_tilt_move_route():
    PassageCondition.pan_deg = request.json['pan_value']
    PassageCondition.tilt_deg = request.json['tilt_value']
    return "OK"


@app.route('/base_move', methods=['POST'])
def move_route():
    print("Pan > " + str(PassageCondition.pan_deg))
    print("Tilt > " + str(PassageCondition.tilt_deg))
    print("Distance > " + str(PassageCondition.target_distance))
    result = move_action.process(pan=math.radians(PassageCondition.pan_deg),
                        tilt=math.radians(PassageCondition.tilt_deg),
                        distance=PassageCondition.target_distance)
    if result == True:
        return jsonify({}), 200
    return 500


@app.route('/base_stop', methods=['POST'])
def stop_route():
    print("stop request")
    return jsonify({}), move_action.stop()
    

@app.route('/', methods=['GET','POST'])
def index():
    return render_template('index.html')


def web_app_background():
    print('start_app')
    app.run(host="localhost", port=8080)


if __name__ == '__main__':
    
    # Web App Start - Run in other thread
    web_app = threading.Thread(target=web_app_background)
    web_app.start()

    # ROS Start - Run in the main thread
    rospy.init_node("web_app")
    pc = PassageCondition()
    rospy.spin()
    


    