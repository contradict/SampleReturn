#!/bin/bash

#MASTER_HOST="blue18"
MASTER_HOST="moline"
#SLAVE_HOSTS="moline"
SLAVE_HOSTS=""

xset dpms force off

. /opt/ros/fuerte/setup.bash
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_PACKAGE_PATH="/home/robot/Desktop/SampleReturn/src/ros:$ROS_PACKAGE_PATH"
export ROS_MASTER_URI="http://${MASTER_HOST}:11311"

echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"

sleep 10

if [ `hostname` = ${MASTER_HOST} ]; then
    if ! pidof python | grep `cat ${HOME}/.ros/roscore-11311.pid 2>/dev/null` >/dev/null 2>&1; then
        roscore &
    fi
    sleep 10
    for machine in ${SLAVE_HOSTS}; do
        until ping -c1 -w5 ${machine} >>/dev/null 2>&1; do
            wakeonlan ${machine}
            sleep 5
        done
    done
	roslaunch --pid=${HOME}/.ros/driving_test.pid samplereturn driving_test.launch &
    # sleep 10
	# roslaunch --pid=${HOME}/.ros/logging.pid samplereturn logging.launch &
else
    until ping -c1 -w5 ${MASTER_HOST} >>/dev/null 2>&1; do
        wakeonlan ${MASTER_HOST}
        sleep 5
    done
fi

