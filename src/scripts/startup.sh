#!/bin/bash

MASTER_HOST="sr1-internal"
PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

cd ~robot/Desktop/SampleReturn
. /opt/ros/groovy/setup.bash
. ros_workspace/install/setup.bash

export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_MASTER_URI="http://${MASTER_HOST}:11311"

if echo ${MASTER_HOST} | grep -q sr1; then

    if ! pidof python | grep `cat ${HOME}/.ros/roscore-11311.pid 2>/dev/null` >/dev/null 2>&1; then
        roscore &
    fi

    #sleep 10
    /usr/bin/aplay /usr/lib/libreoffice/share/gallery/sounds/romans.wav

    roslaunch --pid=${HOME}/.ros/driving_test.pid samplereturn driving_test.launch &
fi

