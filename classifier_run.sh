#!/bin/bash

clear

# Variables
export LASER_TO_KINECT_X="0"
export LASER_TO_KINECT_Y="0"
export LASER_TO_KINECT_Z="0.20"
export MINIMUM_GREEN_POINTS="0.3"

# Starting the classifier publisher
../../devel/lib/robot_cognition_system/passage_classifier_publisher