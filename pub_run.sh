#!/bin/bash

clear

# Variables
export POINT_CLOUD_API="/camera/depth/points"
export UPD_UNEVENNESS="4"
export UPD_UNEVENNESS_MAX="10"
export UPD_RAD_ANGL="15"
export UPD_SEARCH_RADIUS="0.9"
export UPD_VOX_GRID_LEAF_X="0.01"
export UPD_VOX_GRID_LEAF_Y="0.01"
export UPD_VOX_GRID_LEAF_Z="0.01"
export UPD_REDUCTION_PERCENT="25"

# Staring the upd publisher
../../devel/lib/robot_cognition_system/upd_publisher