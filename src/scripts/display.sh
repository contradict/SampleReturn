#!/bin/bash

. /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://sr1:11311
modprobe uvesafb mode_option=1024x768-60
./testpygame.py

