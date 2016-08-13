#!/bin/bash
LAUNCH_FILES="start_manipulator_cameras.launch driving_test.launch"
DELAY=5
PIDPATH=${HOME}/.ros

pidname() {
    echo ${PIDPATH}/$(basename ${1} .launch).pid
}

case ${1} in
    start)
        shift
        for launch in ${LAUNCH_FILES}; do
            roslaunch --pid=$(pidname ${launch}) samplereturn ${launch} "$@" &
            sleep ${DELAY}
        done
        ;;
    stop)
        for launch in ${LAUNCH_FILES}; do
            kill -INT $(cat $(pidname ${launch}))
        done
        ;;
    *)
        echo "Must specify \"start\" or \"stop\" as first argument"
        exit 1
        ;;
esac 
