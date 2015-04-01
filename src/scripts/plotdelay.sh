#!/bin/bash

# time format present in PTPD logs
FMT="%Y-%m-%d %H:%M:%S"
# default 30 min into the past
HISTORY_LENGTH=30
# stopping at present time
END=`date +%s`

# allow begin (in minutes before end)
# and end (as any string recognized by date)
# to be specified on the command line
while getopts "b:e:" opt; do
    case $opt in
        b)
            HISTORY_LENGTH=${OPTARG}
            ;;
        e)
            END=`date --date ${OPTARG} +$S`
            ;;
    esac
done

# compute formatted start and end times
START=$((${END}-${HISTORY_LENGTH}*60))
start_fmt=`date --date @${START} "+${FMT}"`
end_fmt=`date --date @${END} "+${FMT}"`

# copy file from robot
TMPCPY=`tempfile -d /tmp -s_plotdelay.txt`
scp -q robot@sr2:~zlizer/ptpd_output.txt ${TMPCPY}
# strip lines not ending with [ISD], the
# remaining lines all follow the format
# Timestamp, State, Clock ID, One Way Delay, Offset From Master,
#   Slave to Master, Master to Slave, Drift, Discarded Packet Count,
#   Last packet Received
TMPPLT=`tempfile -d /tmp -s_plotdelay.txt`
grep "[ISD$]" ${TMPCPY} >${TMPPLT}

# create gnuplot script
TMPGNU=`tempfile -d /tmp -s_plotdelay.gnuplot`
cat >${TMPGNU} <<__EOF__
set datafile separator ","
set timefmt "${FMT}"
set xdata time
set xrange ["${start_fmt}":"${end_fmt}"]
plot "${TMPPLT}" using 1:5
pause -1
__EOF__

# make the plot
gnuplot ${TMPGNU}

# clean up intermediate files
rm ${TMPCPY} ${TMPPLT} ${TMPGNU}
