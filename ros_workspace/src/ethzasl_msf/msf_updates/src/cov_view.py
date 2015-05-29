#!/usr/bin/env python
import rospy
import numpy as np
from matplotlib import pylab
from sensor_fusion_comm.msg import DoubleMatrixStamped

class CovView(object):
    def __init__(self):
        self.figure = pylab.Figure()
        self.ax = self.figure.add_subplot(111)
        self.image = None
        self.sub = rospy.Subscriber("core_cov", DoubleMatrixStamped,
                self.drawcov)

    def drawcov(self, msg):
        mat = np.array(msg.data).reshape((msg.rows, msg.cols))
        if self.image is None:
            self.image = self.ax.imshow(mat)
        else:
            self.image.set_data(mat)
        self.figure.canvas.draw()
