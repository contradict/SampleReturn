#!/usr/bin/env gnuplot
set multiplot layout 2,1 rowsfirst
set label 1 'positions' at graph 0.1,0.9 font ',8'
plot \
     'test.dat' using 1:8:9 title 'Measured position' with errorbars, \
     'test.dat' using 1:4:5 title 'Estimated position' with errorbars, \
     'test.dat' using 1:2 title 'Position'
set label 1 'velocities' at graph 0.1,0.9 font ',8'
plot \
     'test.dat' using 1:6:7 title 'Estimated velocity' with errorbars, \
     'test.dat' using 1:3 title 'Velocity'
unset multiplot
pause mouse close
