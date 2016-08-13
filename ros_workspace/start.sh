#!/bin/bash
LAUNCH_FILES="start_manipulator_cameras.launch driving_test.launch"
DELAY=10
for launch in ${LAUNCH_FILES}; do
    pidname=`basename ${launch} .launch`.pid
    roslaunch --pid=${HOME}/.ros/${pidname} samplereturn ${launch} "$@" &
    sleep ${DELAY}
done
