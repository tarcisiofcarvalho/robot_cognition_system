#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def transform_laser_pan_tilt_to_XY_2D_in_meters(pan=None, tilt=None, range=None):
    """
        This component will transform the laser pan and tilt orientation values
        to a point XY in the 2D plane    
    """

    print("Pan: {0}".format(pan))
    print("Tilt: {0}".format(tilt))
    print("Range: {0}".format(range.ranges[0]))
    # https://www.mathworks.com/matlabcentral/answers/427558-how-can-i-create-xyz-coordinant-from-pan-tilt-system-angles
    # function [x, y, z] = my_sph2cart(pan,tilt,range)
    #     x = range .* cosd(tilt) .* cosd(pan);
    #     y = range .* cosd(tilt) .* sind(pan);
    #     z = range .* sind(tilt);
    # end
    
    
    return 2, -2

def move(x=None, y=None):
    """ 
        This component will request the ROS move_base to move the robot base to
        a given X and Y value in meters 
    """

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
        
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 2.0
    goal.target_pose.pose.position.y = -2.0

    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    # ros move base slam mapping dynamic map
    # https://answers.ros.org/question/337981/ros-amcl-path-planning-with-live-gmapping-map/
    # http://wiki.ros.org/base_local_planner
    # http://wiki.ros.org/dwa_local_planner
    # http://wiki.ros.org/move_base
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

def process(pan=None, tilt=None, laser_range=None):
    """ 
        Execute the main process of the move to target request 
    """
    try:
        # If the python node is executed as main process (sourced directly)
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        # rospy.init_node('movebase_client_py')
        
        # 1. Convert the laser orientation data to X and Y axis point direction
        x, y = transform_laser_pan_tilt_to_XY_2D_in_meters(pan=pan, 
                                                           tilt=tilt, 
                                                           range=laser_range)

        # 2. Set X and Y meters ahead to /move_base/goal node
        #result = move(x, y)

        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
