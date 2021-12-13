#!/bin/bash

clear

# Variables
export LASER_TO_KINECT_X="0"
export LASER_TO_KINECT_Y="0"
export LASER_TO_KINECT_Z="0.40"
export VOX_GRID_LEAF_X="0.07"
export VOX_GRID_LEAF_Y="0.07"
export VOX_GRID_LEAF_Z="0.07"
export MINIMUM_GREEN_POINTS="0.9"
export ROS_LOOP_RATE="1.5"
export HEIGHT_THRESHOLD="-0.20"
export POINT_CLOUD_TO_ROBOT_CENTER="1.80"
# export LASER_X="0.0"
# export LASER_Y="0.0"
# export LASER_Z="0.0"
export LASER_X="0.50"
export LASER_Y="-0.20"
export LASER_Z="0.15"
export LASER_Z_GAP="0.35"
export GENERATE_PATH_LINE_K="12"
export LASER_TYPE="simulated"
# Starting the classifier publisher
../../devel/lib/robot_cognition_system/passage_classifier_publisher