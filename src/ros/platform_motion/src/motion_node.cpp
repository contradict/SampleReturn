#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion/HomeAction.h>
#include <platform_motion/Enable.h>
#include <platform_motion/GPIO.h>

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

