import rospy
import tf
from samplereturn_msgs.msg import NamedPoint

def print_servo_distance():
    listener = tf.TransformListener()
    def printstuff(msg, listener=listener):
        try:
            pt_in_manip = listener.transformPoint( 'manipulator_arm', msg )
            print pt_in_manip.point.x, pt_in_manip.point.y, pt_in_manip.point.z
        except:
            pass
    sub = rospy.Subscriber("/processes/sample_detection/manipulator/point",
            NamedPoint, printstuff)
    return sub
