#!/bin/bash

xset dpms force off

. /opt/ros/opencv_overlay/overlay_setup.sh
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_PACKAGE_PATH="/home/robot/Desktop/SampleReturn/src/ros:$ROS_PACKAGE_PATH"
export ROS_MASTER_URI="http://blue18:11311"

echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"

sleep 10

if [ `hostname` = "blue18" ]; then
    if ! pidof python | grep `cat ${HOME}/.ros/roscore-11311.pid 2>/dev/null` >/dev/null 2>&1; then
        roscore &
    fi
    sleep 10
    until ping -c1 -w5 moline >>/dev/null 2>&1; do
        wakeonlan moline
        sleep 5
    done
	roslaunch --pid=${HOME}/.ros/driving_test.pid samplereturn driving_test.launch &
    # sleep 10
	# roslaunch --pid=${HOME}/.ros/logging.pid samplereturn logging.launch &
else
    until ping -c1 -w5 blue18 >>/dev/null 2>&1; do
        wakeonlan blue18
        sleep 5
    done
fi

