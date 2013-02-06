#!/bin/sh

. /opt/ros/fuerte/setup.sh
export ROS_PACKAGE_PATH="/home/robot/Desktop/SampleReturn/src/ros:${ROS_PACKAGE_PATH}"

exec "$@"
