#!/bin/bash

if /sbin/start-stop-daemon \ 
    --start \ 
    --user robot \ 
    --chuid robot \ 
    --pidfile ~robot/.ros/roscore-11311.pid \ 
    /home/robot/Desktop/SampleReturn/src/scripts/startup.sh; then 
true 
else 
    /usr/bin/aplay /usr/lib/libreoffice/share/gallery/sounds/strom.wav & 
fi 

