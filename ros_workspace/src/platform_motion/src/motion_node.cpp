#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion_msgs/HomeAction.h>
#include <platform_motion_msgs/Enable.h>
#include <platform_motion_msgs/SelectMotionMode.h>
#include <platform_motion_msgs/GPIO.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <motion/wheelpod.h>
#include <platform_motion/PlatformParametersConfig.h>
#include <motion/motion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PlatformMotion");
    platform_motion::Motion m;
    m.start();
    ROS_INFO("spin");
    ros::spin();
    m.shutdown();
}

