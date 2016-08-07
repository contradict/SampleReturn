#!/bin/bash

MASTER_HOST="sr1"
MASTER_PORT="11311"
OTHER_HOST="sr2"
SSH_PORT="22"
PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
HOME=/home/robot
if [ $# -eq 0 ]; then
    LAUNCH_FILE=driving_test.launch
else
    LAUNCH_FILE=$1
fi

eval `ssh-agent`
ssh-add ${HOME}/.ssh/robot_rsa

cd ${HOME}/Desktop/SampleReturn
. /opt/ros/indigo/setup.bash
. ros_workspace/install/setup.bash

export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_MASTER_URI="http://${MASTER_HOST}:${MASTER_PORT}"

if echo ${MASTER_HOST} | grep -q `hostname`; then

    if ! pidof python | grep `cat ${HOME}/.ros/roscore-11311.pid 2>/dev/null` >/dev/null 2>&1; then
        echo "Starting new core process"
        roscore &
    fi

    # wait for roscore and sr2 to be ready
    until nc -z ${MASTER_HOST} ${MASTER_PORT} && nc -z ${OTHER_HOST} ${SSH_PORT}; do sleep 1; done

    pidname=`basename ${LAUNCH_FILE} .launch`.pid
    roslaunch --pid=${HOME}/.ros/${pidname} samplereturn ${LAUNCH_FILE} &
    roslaunch --pid=${HOME}/.ros/logging.pid samplereturn logging.launch &
else
    if echo ${OTHER_HOST} | grep -q `hostname`; then
        # This requires the line
        # robot ALL=(root) NOPASSWD: /bin/kill
        # in /etc/sudoers
        sudo -n kill -USR1 `pidof ptpd`
    fi
fi
MACHINE=`hostname`
/usr/bin/aplay /home/robot/sounds/${MACHINE}start.wav
