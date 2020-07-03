#!/usr/bin/env python

import rospy 
from std_msgs.msg import String
from flask import Flask, request, jsonify, render_template
import threading

passage_condition_result = "test"
app = Flask(__name__)

class PassageCondition():

    passage_condition = "test"

    def __init__(self):
        self.subscriber = rospy.Subscriber("/passage_condition", String, self.callBack)
        self.passage_condition = "test"

    def callBack(self, data):
        # print(data)
        PassageCondition.passage_condition = data
        # passage_condition = data

@app.route('/passage_condition', methods=['GET'])
def passage_condition_route():
    passage_condition_result = PassageCondition.passage_condition.data
    return jsonify({'condition':str(passage_condition_result)})

@app.route('/', methods=['GET','POST'])
def index():
    return render_template('index.html')


def ros_background():
    print('start_app')
    app.run(host="localhost", port=8080)


if __name__ == '__main__':
    ros_process = threading.Thread(target=ros_background)
    ros_process.start()
    rospy.init_node("web_app")
    pc = PassageCondition()
    # app.run(host="localhost", port=8080)
    rospy.spin()
    


    