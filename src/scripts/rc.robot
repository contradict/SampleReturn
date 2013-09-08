#!/bin/bash

/sbin/start-stop-daemon --user robot --chuid robot --pidfile /home/robot/.ros/roscore-11311.pid --exec /home/robot/Desktop/SampleReturn/src/scripts/startup.sh --start

if [ "$?" != "0" ]; then
    /usr/bin/aplay /usr/lib/libreoffice/share/gallery/sounds/strom.wav &
fi

