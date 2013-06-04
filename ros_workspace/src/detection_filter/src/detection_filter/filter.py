import tf
import rospy
import numpy as np

class Hypothesis(object):
    def __init__(self, position=None, tolerance=0.1, alpha=0.25, unsupported_step=0.5):
        self.position = position
        self.tolerance = tolerance
        self.alpha = alpha
        self.unsupported_step = unsupported_step
        if self.position is not None:
            self.support = 1.0
        else:
            self.support = 0.0

    def supports(self, position):
        if self.position is None:
            return True
        return self.distance_to(position)<self.tolerance

    def distance_to(self, position):
        return np.sqrt(((self.position[:2]-position[:2])**2).sum())

    def update(self, position):
        if self.position is None:
            self.position = position
            self.support = 1.0
        else:
            self.position = (1.0-self.alpha)*self.position + self.alpha*position
            self.support += 1.0

    def unsupported(self):
        if self.support > 0.0:
            self.support -= self.unsupported_step
        if self.support <= 0.0:
            self.support = 0.0
            self.position = None

    def valid(self, threshold):
        return self.support>threshold

    def replace(self, other):
        self.position = other.position
        self.support = other.support

    def __lt__(self, other):
        return self.support<other.support

    def __call__(self, position=None):
        if position is not None:
            position = position.copy()
        h=Hypothesis(position=position, tolerance=self.tolerance, alpha=self.alpha,
                unsupported_step=self.unsupported_step)
        return h

    def __str__(self):
        return "%s: %3.1f"%(self.position, self.support)

    def __repr__(self):
        return self.__str__()

class Filter(object):
    def __init__(self, hypothesis, frame="map", hypotheses=3):
        self.frame = frame
        self.hypothesis = hypothesis

        self.hypotheses=[self.hypothesis() for x in xrange(hypotheses)]

        self.listener = tf.TransformListener()

    def update(self, msg, threshold, unsupport=False):
        latest = rospy.Time(0)
        if not self.listener.canTransform(self.frame, msg.header.frame_id, latest):
            rospy.logerr("Can not transform frame %s to %s, ignoring measurement",
                    msg.header.frame_id, self.frame)
            return
        tf_pt = self.listener.transformPoint(self.frame, msg)
        point = np.array([tf_pt.point.x, tf_pt.point.y, tf_pt.point.z])
        worst=self.hypotheses[0]
        result = None
        for h in self.hypotheses:
            if h.supports(point):
                rospy.logdebug("update %s with %s", h, point)
                h.update(point)
                result = h if h.support>threshold else None
                break
            if h<worst:
                worst = h
        else:
            worst.replace(self.hypothesis(point))
        if unsupport:
            for h in self.hypotheses:
                if not h.supports(point):
                    rospy.logdebug("unsupported %s: %s", h, point)
                    h.unsupported()
                h.unsupported()
        rospy.logdebug("hypotheses: %s", ["%s"%x for x in self.hypotheses])
        return result

    def decay(self):
        rospy.logdebug("decay")
        for h in self.hypotheses:
            h.unsupported()
        rospy.logdebug("hypotheses: %s", ["%s"%x for x in self.hypotheses])

    def estimate(self, threshold=None):
        best = self.hypotheses[0]
        if best is None:
            return None
        for h in self.hypotheses:
            if best<h:
                best = h
        if threshold is None or best.valid(threshold):
            return best
        else:
            return None

