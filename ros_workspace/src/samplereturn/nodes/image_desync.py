#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CameraInfo
import rospy
import rosnode
import os
import time
import rosnode

def get_argv(pid):
    try:
        cmdl = open("/proc/%d/cmdline"%pid).read()
        rospy.logdebug("Reading cmdline for pid %d: %s", pid, cmdl)
    except Exception,e:
        rospy.logerr("Unable to read cmdline for pid %d: %s", pid, e);
        return []
    return cmdl.split("\x00")

def get_envvar(pid, name):
    try:
        env = open("/proc/%d/environ"%pid).read()
    except Exception, e:
        rospy.logerr("Unable to read environ for pid %d: %s", pid, e)
        return None
    for envvar in env.split("\x00"):
        if not '=' in envvar:
            continue
        var,val = envvar.split("=")[:2]
        if var == name:
            return val
    return None

def get_rosparam(argv, pname):
    rospy.logdebug("Looking for %s in %s"%(pname,argv))
    for arg in argv:
        if ":=" in arg:
            var,val = arg.split(":=")
            if var == pname:
                rospy.logdebug("Found %s", val)
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
            rospy.logerr("killing %d", pid)
            os.system('kill -9 %d'%pid)
            return True
    return False

def masacre_nodelets_in_namespace(namespace):
    _, stdout = os.popen2(['pidof', 'nodelet'])
    pid_str = stdout.read()
    pids = [(int(s),
             get_rosparam(get_argv(int(s)), "__name"),
             get_envvar(int(s), "ROS_NAMESPACE")) for s in pid_str.split()]
    rospy.logdebug("nodelet processes: %s", pids)
    rospy.logdebug("Looking for processes in namespace %s", namespace)
    pids = filter(lambda x: x[1] is not None, pids)
    pids = filter(lambda x: x[2] == namespace, pids)
    rospy.logdebug("Resulting processes: %s", pids)
    if len(pids) == 0:
        return False
    killlevels = ["-INT", "-TERM", "-KILL"]
    while len(pids) and len(killlevels):
        rospy.loginfo("Still running: %s", pids)
        sig = killlevels.pop(0)
        livepids = []
        for pid, name, ns in pids:
            rospy.logerr("Killing %s pid %d, %s/%s", sig, pid, ns, name)
            os.system('kill %s %d'%(sig, pid))
        time.sleep(0.1)
        for pid, name, ns in pids:
            retcode=os.system('kill -0 %d'%pid)
            rospy.logdebug("Checking for existence of %d %d", pid, retcode)
            if retcode == 0:
                rospy.logerr("pid %d %s/%s still running after kill %s",
                             pid, ns, name, sig)
                livepids.append((pid, name, ns))
        pids = livepids
    if len(pids):
        rospy.logerr("Some processes remain after kill -KILL")
    return True

class image_desync(object):
    def __init__(self):
        self.timestamps={'left':[], 'right':[]}
        self.pub = rospy.Publisher('desync', Float64, queue_size = 10)

        self.status_pub = rospy.Publisher('status', String, queue_size = 10)

        self.check_interval    = rospy.get_param("~check_interval",     1.0)
        self.max_desync        = rospy.get_param("~max_desync",         0.10)
        self.restart_on_desync = rospy.get_param("~restart_on_desync",  False)
        self.restart_on_missing= rospy.get_param("~restart_on_missing", True)
        self.max_desync_count  = rospy.get_param("~max_desync_count",  10)
        self.max_missing_count = rospy.get_param("~max_missing_count", 10)
        self.startup_delay     = rospy.get_param("~startup_delay", 30)

        self.manager_node_name = rospy.get_param("~manager_node_name")
        self.namespace = get_envvar(os.getpid(), "ROS_NAMESPACE")

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

        rospy.Subscriber('left/camera_info', CameraInfo,
                lambda info, name='left': self.info_callback(name, info))
        rospy.Subscriber('right/camera_info', CameraInfo,
                lambda info, name='right': self.info_callback(name, info))

        self.startup_time = rospy.Time.now()

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
        if not (self.got_info['left'] or self.got_info['right']):
            self.pub.publish(Float64(float('nan')))
            self.missing_count['left']  += 1
            self.missing_count['right'] += 1
        elif not self.got_info['left']:
            self.pub.publish(Float64(-float('inf')))
            self.missing_count['left']  += 1
            self.missing_count['right']  = 0
        elif not self.got_info['right']:
            self.pub.publish(Float64(float('inf')))
            self.missing_count['left']   = 0
            self.missing_count['right'] += 1
        else:
            self.missing_count['left'] = 0
            self.missing_count['right'] = 0
        self.got_info['left'] = False
        self.got_info['right'] = False
        self.check_missing()

    def restart_manager(self, message, restart="both"):
        remaining_time =  self.startup_delay - (rospy.Time.now() - self.startup_time).to_sec()
        if remaining_time>0.0:
            rospy.logdebug("Waiting to restart: %f", remaining_time)
            return
        if restart == "both":
            rospy.logerr("%s, restarting manager", message)
            self.desync_count = 0
            # node must be marked respawn in launch file,
            #if not kill_nodelet_manager(self.manager_node_name):
            #    rospy.logerr("Unable to kill manager %s", self.manager_node_name)
            if not masacre_nodelets_in_namespace(self.namespace):
                rospy.logerr("Unable to kill any nodes in this namespace: %s",
                        self.namespace)
        elif restart == "left" or restart == "right":
            rosnode.kill_nodes([self.namespace+"/"+restart])
        self.startup_time = rospy.Time.now()

    def check_desync(self, delta):
        if abs(delta)>self.max_desync:
            self.desync_count += 1
        else:
            self.desync_count = 0
            self.status_pub.publish(String("Ready"))
        if self.desync_count > self.max_desync_count:
            self.status_pub.publish(String("Desynchronized"))
            if self.restart_on_desync:
                self.restart_manager("Cameras Desynchronized")

    def check_missing(self):
        if self.missing_count['left'] > self.max_missing_count and \
           self.missing_count['right'] > self.max_missing_count:
            self.status_pub.publish(String("Both Missing"))
            if self.restart_on_missing:
                self.restart_manager("Cameras failed")
        elif self.missing_count['left'] > self.max_missing_count:
            self.status_pub.publish(String("Left Missing"))
            if self.restart_on_missing:
                self.restart_manager("Left camera failed", restart="left")
        elif self.missing_count['right'] > self.max_missing_count:
            self.status_pub.publish(String("Right Missing"))
            if self.restart_on_missing:
                self.restart_manager("Right camera failed", restart="right")

if __name__=="__main__":
    rospy.init_node('image_desync')
    id = image_desync()
    rospy.spin()


