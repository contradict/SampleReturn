#!/bin/bash

. /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=/home/robot/Desktop/SampleReturn/src/ros:$ROS_PACKAGE_PATH
roscore &

sleep 5

roslaunch samplereturn driving_test.launch &
#roslaunch samplereturn logging.launch &

