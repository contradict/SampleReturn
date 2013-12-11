#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CameraInfo
import rospy
import rosnode
import os

def get_argv(pid):
    try:
        cmdl = open("/proc/%d/cmdline"%pid).read()
    except Exception,e:
        return []
    return cmdl.split("\x00")

def get_envvar(pid, name):
    try:
        env = open("/proc/%d/environ"%pid).read()
    except:
        return None
    for envvar in env.split("\x00"):
        if not '=' in envvar:
            continue
        var,val = envvar.split("=")[:2]
        if var == name:
            return val
    return None

def get_rosparam(argv, pname):
    for arg in argv:
        if ":=" in arg:
            var,val = arg.split(":=")
            if var == pname:
                return val
    return None

def kill_nodelet_manager(managername):
    stdin, stdout = os.popen2(['pidof', 'nodelet'])
    pid_str = stdout.read()
    pids = [int(s) for s in pid_str.split()]
    for pid in pids:
        argv = get_argv(pid)
        if len(argv)<2:
            continue
        if argv[1] != "manager":
            continue
        name=get_rosparam(argv[2:], "__name")
        if name is None:
            continue
        ns=get_envvar(pid, "ROS_NAMESPACE")
        if ns is None:
            testname="/%s"%name
        else:
            testname="%s/%s"%(ns,name)
        if testname==managername:
            os.system('kill -9 %d'%pid)
            return True
    return False


class image_desync(object):
    def __init__(self):
        self.timestamps={'left':[], 'right':[]}
        self.pub = rospy.Publisher('desync', Float64)
        rospy.Subscriber('left/camera_info', CameraInfo,
                lambda info, name='left': self.info_callback(name, info))
        rospy.Subscriber('right/camera_info', CameraInfo,
                lambda info, name='right': self.info_callback(name, info))

        self.status_pub = rospy.Publisher('status', String)

        self.check_interval    = rospy.get_param("~check_interval",     1.0)
        self.max_desync        = rospy.get_param("~max_desync",         0.10)
        self.max_desync_count  = rospy.get_param("~max_desync_count",  10)
        self.max_missing_count = rospy.get_param("~max_missing_count", 10)
        self.sync_wait         = rospy.get_param("~sync_wait",          5.0)
        self.manager_node_name = rospy.get_param("~manager_node_name")

        self.timer = rospy.Timer(rospy.Duration(self.check_interval),
                self.check_for_data)

        # not really, but this gives a check_interval
        # pause before the first error message
        self.got_info={'left':True,
                       'right':True
                       }
        self.desync_count = 0
        self.missing_count={'left':0,
                            'right':0
                            }
        self.sync_wait_timer = None

    def info_callback(self, name, info):
        self.got_info[name] = True
        self.timestamps[name].append(info.header.stamp)
        if len(self.timestamps[name]) >2:
           self.timestamps[name].pop(0)
        if name=='left' and all([len(x)==2 for x in self.timestamps.itervalues()]):
            delta1 = (self.timestamps['left'][0] -
                    self.timestamps['right'][1]).to_sec()
            delta2 = (self.timestamps['left'][1] -
                self.timestamps['right'][1]).to_sec()
            if abs(delta1)<abs(delta2):
                delta = delta1
            else:
                delta = delta2
            self.check_desync(delta)
            self.pub.publish(Float64(delta))
            self.timestamps['left'].pop(0)
            self.timestamps['right'].pop(0)

    def check_for_data(self, event):
        if self.sync_wait_timer is not None:
            return
        if not (self.got_info['left'] or self.got_info['right']):
            self.pub.publish(Float64(float('nan')))
            self.missing_count['left']  += 1
            self.missing_count['right'] += 1
        elif not self.got_info['left']:
            self.pub.publish(Float64(-float('inf')))
            self.missing_count['left']  += 1
        elif not self.got_info['right']:
            self.pub.publish(Float64(float('inf')))
            self.missing_count['right'] += 1
        else:
            self.missing_count['left'] = 0
            self.missing_count['right'] = 0
        self.got_info['left'] = False
        self.got_info['right'] = False
        self.check_missing()

    def restart_manager(self):
        self.desync_count = 0
        # node must be marked respawn in launch file,
        if not kill_nodelet_manager(self.manager_node_name):
            rospy.logerr("Unable to kill manager %s", self.manager_node_name)
        if self.sync_wait_timer is None:
            self.sync_wait_timer = rospy.Timer(rospy.Duration(self.sync_wait),
                        self.sync_wait_timeout, oneshot=True)

    def sync_wait_timeout(self, event):
        rospy.logerr("Camera resync timeout, restarting manager again")
        self.sync_wait_timer = None
        self.restart_manager()

    def check_desync(self, delta):
        if abs(delta)>self.max_desync:
            self.desync_count += 1
        else:
            if self.sync_wait_timer is not None:
                rospy.logwarn("Cameras resynchronized")
                self.sync_wait_timer.shutdown()
                self.sync_wait_timer = None
            self.desync_count = 0
            self.status_pub.publish(String("Ready"))
        if self.desync_count > self.max_desync_count:
            self.status_pub.publish(String("Desynchronized"))
            rospy.logerr("Cameras desynchronized, restarting manager")
            self.restart_manager()

    def check_missing(self):
        if self.missing_count['left'] > self.max_missing_count and \
           self.missing_count['right'] > self.max_missing_count:
            rospy.logerr("Cameras failed, restarting manager")
            self.status_pub.publish(String("Both Missing"))
            self.restart_manager()
        elif self.missing_count['left'] > self.max_missing_count:
            rospy.logerr("Left camera failed, restarting manager")
            self.status_pub.publish(String("Left Missing"))
            self.restart_manager()
        elif self.missing_count['right'] > self.max_missing_count:
            rospy.logerr("Right camera failed, restarting manager")
            self.status_pub.publish(String("Right Missing"))
            self.restart_manager()

if __name__=="__main__":
    rospy.init_node('image_desync')
    id = image_desync()
    rospy.spin()


