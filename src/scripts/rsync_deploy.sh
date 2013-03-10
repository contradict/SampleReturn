#!/bin/bash

mypath=`dirname $0`
workspacepath=${mypath}/../../ros_workspace
installpath=Desktop/SampleReturn/ros_workspace/install/

pushd ${workspacepath}
catkin_make \
    -DCMAKE_INSTALL_PREFIX=/home/robot/Desktop/SampleReturn/ros_workspace/install \
    -DCMAKE_BUILD_TYPE="Release" \
    install
popd

while (( "$#" )); do
    target=$1
    echo "send to $target"
    rsync -av ~robot/${installpath} robot@${target}:${installpath}
    shift
done
