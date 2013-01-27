#!/bin/bash

remote=$1
if [ -n "${2}" ]; then
    branch=${2}
else
    branch=$(git describe --contains --all HEAD)
fi
thisdir=`dirname $0`

git push -f ${remote} ${branch} || exit 1

rmt=$(git remote -v | awk "/^${remote}\t.*\(push\)\$/ {print \$2}")
account=$(echo ${rmt} | cut -d":" -f1)
host=$(echo ${account} | cut -d"@" -f2)
rmtdir=$(echo ${rmt} | cut -d":" -f2)

ssh -n ${account} \
    "cd ${rmtdir}; \
     git checkout --detach ${branch} || exit 1; \
     git reset --hard ${branch} || exit 1; \
     git submodule update -f --init || exit 1; \
     pushd ros_workspace 2>&1 >>/dev/null || exit 1; \
     . /opt/ros/groovy/setup.bash; \
     catkin_init_workspace src 2>&1 >>/dev/null && true; \
     catkin_make --force-cmake install || exit 1; \
     popd 2>&1 >>/dev/null; \
     if [ \`hostname\` = 'sr1' ]; then \
        if ping -c1 -w1 sr2-internal 2>&1 >>/dev/null; then \
            git remote | grep sr2-internal 2>&1 >>/dev/null || \
                git remote add sr2-internal robot@sr2-internal:Desktop/SampleReturn; \
            ${0} sr2-internal ${branch} || exit 1; \
        fi; \
     fi"
