#!/bin/bash

clear

# Variables
export POINT_CLOUD_API="/camera/depth/points"
export UPD_UNEVENNESS="8"
export UPD_UNEVENNESS_MAX="10"
export UPD_DEG_ANGL="20"
export UPD_SEARCH_RADIUS="0.4"
export UPD_VOX_GRID_LEAF_X="0.03"
export UPD_VOX_GRID_LEAF_Y="0.03"
export UPD_VOX_GRID_LEAF_Z="0.03"
export UPD_REDUCTION_PERCENT="25"
export KINECT_TO_BASE_X="0.50"
export KINECT_TO_BASE_Y="0.2"
export KINECT_TO_BASE_Z="0.15"
export ROS_UPD_LOOP_RATE="0.9"
  
# Starting the upd publisher
../../devel/lib/robot_cognition_system/upd_publisher
# Starting the laser pointer publisher
# python src/laser_publisher.py