#!/bin/bash

clear

# Variables
export LASER_TO_KINECT_X="0"
export LASER_TO_KINECT_Y="-0.40"
export LASER_TO_KINECT_Z="0"
export VOX_GRID_LEAF_X="0.01"
export VOX_GRID_LEAF_Y="0.01"
export VOX_GRID_LEAF_Z="0.01"
export MINIMUM_GREEN_POINTS="0.9"
export ROS_LOOP_RATE="1.5"
export HEIGHT_THRESHOLD="-0.20"
export POINT_CLOUD_TO_ROBOT_CENTER="1.61"
# export LASER_X="0.0"
# export LASER_Y="0.0"
# export LASER_Z="0.0"
export LASER_X="-0.25" # 0.5
export LASER_Y="-0.25" # -0.20
export LASER_Z="1.45" # 0.15
export LASER_Z_GAP="0.0" # 0.35
export GENERATE_PATH_LINE_K="100"
export LASER_TYPE="simulated"
export ROBOT_BASE_X="0.30"  
export ROBOT_BASE_Y="0.0" 
export ROBOT_BASE_Z="0.0" 
# Starting the classifier publisher
../../devel/lib/robot_cognition_system/passage_classifier_publisher