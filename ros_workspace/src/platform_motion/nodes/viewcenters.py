#!/usr/bin/python
import Queue
import rospy
import sys
from platform_motion.msg import Centers
from std_msgs.msg import Float64

import wxmpl

class ViewCenters(object):
    def __init__(self, app):
        self.app=app
        self.figure=self.app.get_figure()
        self.ax=self.figure.gca()
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        self.desired_center, = self.ax.plot([0], [0], 'rx')
        self.present_center, = self.ax.plot([0], [0], 'gx')
        self.interpolated_center, = self.ax.plot([0], [0], 'bx')
        self.mq = Queue.Queue(1)
        self.app.Bind(wxmpl.wx.EVT_IDLE, self.OnIdle)
        rospy.Subscriber("/debug", Centers, self.centers)

    def draw(self, point, vect):
        if vect.z == 0:
            point.set_data(
                    [0.0, 5.0*vect.x],
                    [0.0, 5.0*vect.y]
                    )
            point.set_linestyle('-')
            point.set_marker('')
        else:
            point.set_data(
                    [vect.x],
                    [vect.y]
                    )
            point.set_linestyle('')
            point.set_marker('x')

    def centers(self, msg):
        try:
            self.mq.put_nowait(msg)
            wxmpl.wx.WakeUpIdle()
        except Queue.Full:
            pass

    def OnIdle(self, evt):
        if not self.mq.empty():
            msg = self.mq.get()
            for point, vect in zip(
                    (self.desired_center, self.present_center, self.interpolated_center),
                    (msg.desired_center, msg.present_center, msg.interpolated_center)):
                self.draw(point, vect)
            self.figure.canvas.draw()
        if rospy.is_shutdown():
            self.app.Exit()

if __name__=="__main__":
    rospy.init_node('view_centers')
    app=wxmpl.PlotApp('Centers')
    v=ViewCenters(app)
    app.MainLoop()


