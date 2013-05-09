#include <string>

#include <Eigen/Dense>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "motion/acceleration_limit.h"

class VelocityLimiter
{
    public:
        VelocityLimiter();

    private:
        int lookupPodPosition(std::string name, Eigen::Vector2d *pos);
        void handleTwist(const geometry_msgs::Twist::ConstPtr twist);
        void handleOdometry(const nav_msgs::Odometry::ConstPtr odo);

        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Publisher limited_twist;
        tf::TransformListener listener;
        ros::Subscriber st;
        ros::Subscriber so;

        Eigen::Vector2d stern_position, starboard_position, port_position;

        boost::mutex velocity_mutex;
        Eigen::Vector3d body_velocity;

        std::string body_point_name;

        double max_steering_omega;

        Eigen::Vector3d deltav;
};

VelocityLimiter::VelocityLimiter() :
    pnh("~")
{
    double delta_vx, delta_vy, delta_w;
    pnh.param<double>("max_steering_omega", max_steering_omega, 0.1);
    pnh.param<double>("delta_vx", delta_vx, 0.01);
    pnh.param<double>("delta_vy", delta_vy, 0.01);
    pnh.param<double>("delta_w", delta_w, 0.001);
    deltav << delta_vx, delta_vy, delta_w;
    pnh.param<std::string>("body_point_name", body_point_name, "base_link");
    lookupPodPosition(std::string("port_suspension"), &port_position);
    lookupPodPosition(std::string("starboard_suspension"), &starboard_position);
    lookupPodPosition(std::string("stern_suspension"), &stern_position);
    st = nh.subscribe("twist", 1, &VelocityLimiter::handleTwist, this);
    so = nh.subscribe("odometry", 1, &VelocityLimiter::handleOdometry, this);
    limited_twist = nh.advertise<geometry_msgs::Twist>("limited_twist", 1);
}

int VelocityLimiter::lookupPodPosition(std::string name, Eigen::Vector2d *pos)
{
    tf::StampedTransform pod_tf;
    ros::Time current(0);
    ros::Duration timeout(30);
    listener.waitForTransform(body_point_name, name, current, timeout);
    try {
        listener.lookupTransform(body_point_name, name, current, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", name.c_str(), ex.what());
        return -1;
    }
    //ROS_DEBUG("%s at (%f, %f)", joint_name, port_tf.getOrigin().x(), port_tf.getOrigin().y());
    (*pos) << pod_tf.getOrigin().x(), pod_tf.getOrigin().y();
    return 0;
}

void VelocityLimiter::handleTwist(const geometry_msgs::Twist::ConstPtr twist)
{
    boost::unique_lock<boost::mutex> velocity_lock(velocity_mutex);
    Eigen::Vector3d vel_in;
    vel_in << twist->linear.x, twist->linear.y, twist->angular.z;

    Eigen::Vector3d vel_out = platform_motion::SelectClosestVelocity(
        body_velocity,
        stern_position,
        starboard_position,
        port_position,
        max_steering_omega,
        deltav,
        vel_in);

    geometry_msgs::Twist lt;
    lt.linear.x = vel_out(0);
    lt.linear.y = vel_out(1);
    lt.angular.z = vel_out(2);
    limited_twist.publish(lt);
}

void VelocityLimiter::handleOdometry(const nav_msgs::Odometry::ConstPtr odo)
{
    boost::unique_lock<boost::mutex> velocity_lock(velocity_mutex);

    body_velocity << odo->twist.twist.linear.x, odo->twist.twist.linear.y, odo->twist.twist.angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VelocitySmoother");

    VelocityLimiter limit;

    ROS_INFO("spin");
    ros::spin();
}
