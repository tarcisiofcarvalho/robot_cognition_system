#!/bin/bash

clear

# Open catkin source
cd ../../../

# Creating make files
catkin_make -DCATKIN_WHITELIST_PACKAGES="sim_model"