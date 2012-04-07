#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion/HomeWheelPodsAction.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <motion/wheelpod.h>
#include <motion/motion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PlatformMotion");
    platform_motion::Motion m;
    m.start();
    ROS_INFO("spin");
    ros::spin();
    //m.shutdown();
}

