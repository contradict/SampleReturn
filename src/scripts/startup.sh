#!/bin/bash

MASTER_HOST="sr1"
MASTER_PORT="11311"
OTHER_HOST="sr2"
SSH_PORT="22"
PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
HOME=/home/robot
DELAY=5
PTPD_OUTPUT=/home/zlizer/ptpd_output.txt

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

    ros_workspace/robot start log
else
    if echo ${OTHER_HOST} | grep -q `hostname`; then
        # wait for ptpd to start synchronizing
        tail -fn0 ${PTPD_OUTPUT} | grep --line-buffered 'slv' | head -n 10 >>/dev/null
        # force step of local ptpd
        # This requires the line
        # robot ALL=(root) NOPASSWD: /bin/kill
        # in /etc/sudoers
        sudo -n kill -USR1 `pidof ptpd`
    fi
fi
MACHINE=`hostname`
/usr/bin/aplay /home/robot/sounds/${MACHINE}start.wav
