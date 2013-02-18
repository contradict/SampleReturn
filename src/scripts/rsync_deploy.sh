#!/bin/bash

mypath=`dirname $0`
workspacepath=${mypath}/../../ros_workspace
installpath=Desktop/SampleReturn/ros_workspace/install/

pushd ${workspacepath}
catkin_make install
popd

while (( "$#" )); do
    target=$1
    echo "send to $target"
    rsync -av ~robot/${installpath} robot@${target}:${installpath}
    shift
done
