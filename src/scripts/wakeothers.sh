#!/bin/sh

hosts="blue18 moline"

pingdelay=10

wakeonlan=/usr/bin/wakeonlan

wakeup () {
for host in $hosts; do
    echo "Waking $host"
    $wakeonlan $host
done

allawake="false"
while [ ! "$allawake" = "true" ]; do
    sleep $pingdelay
    allawake="true"
    for host in $hosts; do
        if ! ping -c1 -i0.2 $host >/dev/null 2>&1; then
            echo "Waking $host"
            allawake="false"
            $wakeonlan $host
        fi
    done
done

echo "All awake!"
}

shutdown() {
/usr/bin/ssh -i /home/pi/.ssh/id_rsa robot@blue18 ./halt.sh
}

case $1 in
    "start" ) wakeup ;;
    "stop" )
        shutdown
        /sbin/halt -p
    ;;
    * )
        echo "usage: $0 start|stop"
    ;;
esac
