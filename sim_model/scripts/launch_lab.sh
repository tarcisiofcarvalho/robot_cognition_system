#!/bin/bash

clear

# Staring the upd publisher
# roslaunch robot_cognition_system display.launch model:=../urdf/01-myfirst.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/02-multipleshapes.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/03-origins.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/04-materials.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/05-visual.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/06-flexible.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/07-physics.urdf
# roslaunch robot_cognition_system display.launch model:=../urdf/08-macroed.urdf.xacro
# roslaunch robot_cognition_system gazebo.launch
# roslaunch robot_cognition_system display.launch model:=../urdf/sim_model.urdf.xacro
# rosrun rviz rviz
roslaunch openni_launch openni.launch &
roslaunch robot_cognition_system robot_lab.launch &
# ../../upd_run.sh &
# ../../classifier_run.sh &
# python ../src/diode_laser_service.py &
# python ../src/web_app.py
