#!/bin/bash

clear

# Open upd source
cd src/upd_lib/build

# Cleaning up the current build data
rm -rf *

cmake ../

make
