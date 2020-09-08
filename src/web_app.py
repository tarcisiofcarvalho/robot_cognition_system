#!/usr/bin/env python

import rospy 
from std_msgs.msg import String, Float64
from flask import Flask, request, jsonify, render_template
import threading
import math
import move_to_target as move_action

passage_condition_result = "test"

app = Flask(__name__)

class PassageCondition():

    passage_condition = "test"
    pan_deg = 0
    tilt_deg = 0

    def __init__(self):
        self.subscriber = rospy.Subscriber("/passage_condition", String, self.passage_classification)
        self.passage_condition = "test"
        self.laser_pan_tilt_set()

    def passage_classification(self, data):
        # print(data)
        PassageCondition.passage_condition = data
        # passage_condition = data

    def laser_pan_tilt_set(self):
        pub_laser_pan = rospy.Publisher('/r2d2_laser_pan_controller/command', Float64, queue_size=10)
        pub_laser_tilt = rospy.Publisher('/r2d2_laser_tilt_controller/command', Float64, queue_size=10)
        rate = rospy.Rate(100) # 10000hz
        while not rospy.is_shutdown():
            # rospy.loginfo("Pan_rad: " + str(math.radians(self.pan_deg)))
            # rospy.loginfo("Tilt_rad: " + str(math.radians(self.tilt_deg)))
            pub_laser_pan.publish(math.radians(self.pan_deg))
            pub_laser_tilt.publish(math.radians(self.tilt_deg))
            rate.sleep()

@app.route('/passage_condition', methods=['GET'])
def passage_condition_route():
    passage_condition_result = PassageCondition.passage_condition.data
    print("Passage condition: " + str(passage_condition_result))
    return jsonify({'condition':str(passage_condition_result)})

@app.route('/laser_move', methods=['POST'])
def pan_tilt_move_route():
    PassageCondition.pan_deg = request.json['pan_value']
    PassageCondition.tilt_deg = request.json['tilt_value']
    # print("pan_deg: " + str(request.json['pan_value']))
    # print("tilt_deg: " + str(request.json['tilt_value']))
    return "OK"


@app.route('/move', methods=['POST'])
def move_route():
    PassageCondition.pan_deg
    PassageCondition.tilt_deg

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
    


    