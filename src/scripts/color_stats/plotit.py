import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Qt4Agg")
import matplotlib.pyplot as plt
import itertools
from mpl_toolkits.mplot3d import Axes3D

#fig = plt.figure()
#ax = fig.add_subplot(111)

def load_lines(filename):
    lines = []

    with open(filename) as csv:
        for line in csv:
            try:
                (label, imgname, mean_saturation, mean_high_saturation, mean_value, hue_fraction, dominant_hue,
                        saturated_pixel, mean_l, mean_a, mean_b, sat_fraction) = [x.strip() for x in line.split(",")]
            except:
                break
            lines.append([label, imgname]+[float(x) for x in [mean_saturation,
                mean_high_saturation, mean_value,
                hue_fraction, dominant_hue, saturated_pixel, mean_l, mean_a, mean_b, sat_fraction]])
    return lines

def make_pd(lines):
    pd = {}
    pd = defaultdict(list)
    for line in lines:
        pd[line[0]].append(line[2:])
    for k in pd.keys():
        pd[k] = np.array(pd[k])
    return pd

def load(filename='histdata.csv'):
    lines = load_lines(filename)
    pd = make_pd(lines)
    return lines, pd

lines, pd = load()
print "Loaded"

#marker = itertools.cycle(('.','+','*','o'))
#plt.figure(1)
#plt.clf()
#for k in pd.keys():
#    sat = pd[k][:,0]
#    high_sat = pd[k][:,1]
#    plt.plot(sat, pd[k][:,4], marker='.', linestyle='', label=k)
#plt.xlabel("Saturation")
#plt.ylabel("Hue")
#plt.legend()
#plt.draw()
#
#plt.figure(2)
#plt.clf()
#for k in pd.keys():
#    plt.plot(pd[k][:,2], pd[k][:,4], marker='.', linestyle='', label=k)
#plt.xlabel("Value")
#plt.ylabel("Hue")
#plt.legend()
#plt.draw()
#
#plt.figure(3)
#plt.clf()
#for k in pd.keys():
#    sat = pd[k][:,0]
#    high_sat = pd[k][:,1]
#    plt.plot(high_sat, pd[k][:,2], marker='.', linestyle='', label=k)
#plt.xlabel("Saturation")
#plt.ylabel("Value")
#plt.legend()
#plt.draw()

plt.figure(4)
plt.clf()
for k in pd.keys():
    plt.plot(pd[k][:,7], pd[k][:,8], marker='.', linestyle='', label=k)
plt.xlabel("Mean A")
plt.ylabel("Mean B")
plt.legend()
plt.draw()

plt.figure(5)
plt.clf()
for k in pd.keys():
    plt.plot(pd[k][:,7], pd[k][:,6], marker='.', linestyle='', label=k)
plt.xlabel("Mean A")
plt.ylabel("Mean L")
plt.legend()
plt.draw()

plt.figure(6)
plt.clf()
for k in pd.keys():
    plt.plot(pd[k][:,8], pd[k][:,6], marker='.', linestyle='', label=k)
plt.xlabel("Mean B")
plt.ylabel("Mean L")
plt.legend()
plt.draw()

