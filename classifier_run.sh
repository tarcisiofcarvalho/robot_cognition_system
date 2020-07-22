#!/bin/bash

clear

# Variables
export LASER_TO_KINECT_X="0"
export LASER_TO_KINECT_Y="0"
export LASER_TO_KINECT_Z="0.20"
export VOX_GRID_LEAF_X="0.07"
export VOX_GRID_LEAF_Y="0.07"
export VOX_GRID_LEAF_Z="0.07"
export MINIMUM_GREEN_POINTS="0.7"
export LASER_TILT="0.3"
export LASER_PAN="-0.2"
export ROS_LOOP_RATE="2"

# Starting the classifier publisher
../../devel/lib/robot_cognition_system/passage_classifier_publisher