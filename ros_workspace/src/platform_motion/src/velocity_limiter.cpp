#include <string>

#include <Eigen/Dense>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <platform_motion/Centers.h>

#include "motion/acceleration_limit.h"

class VelocityLimiter
{
    public:
        VelocityLimiter();

    private:
        int lookupPodPosition(std::string name, Eigen::Vector2d &pos);
        void handleTwist(const geometry_msgs::Twist::ConstPtr twist);
        void handleOdometry(const nav_msgs::Odometry::ConstPtr odo);
        void handleJointState(const sensor_msgs::JointState::ConstPtr odo);
        bool lookupJoint(const sensor_msgs::JointState::ConstPtr joints, std::string jointName, double *angle, double *velocity);

        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Publisher limited_twist;
        ros::Publisher debug;
        tf::TransformListener listener;
        ros::Subscriber st;
        ros::Subscriber so;
        ros::Subscriber sj;

        Eigen::Vector2d stern_position, starboard_position, port_position;

        boost::mutex velocity_mutex;
        Eigen::Vector3d body_velocity;

        boost::mutex angle_mutex;
        double stern_angle, starboard_angle, port_angle;
        double stern_velocity, starboard_velocity, port_velocity;

        std::string body_point_name;

        double max_steering_omega;
        double twist_period;
        Eigen::Vector3d deltav;
        double tolerance;
        double min_step;
        double wheel_diameter;
};

VelocityLimiter::VelocityLimiter() :
    pnh("~")
{
    pnh.param<double>("max_steering_omega", max_steering_omega, 0.1);
    pnh.param<double>("twist_period", twist_period, 0.067);
    pnh.param<std::string>("body_point_name", body_point_name, "base_link");
    double delta_vx, delta_vy, delta_omega;
    pnh.param<double>("delta_vx", delta_vx, 0.1);
    pnh.param<double>("delta_vy", delta_vy, 0.1);
    pnh.param<double>("delta_omega", delta_omega, 0.015);
    deltav << delta_vx, delta_vy, delta_omega;
    pnh.param<double>("tolerance", tolerance, 0.0001);
    pnh.param<double>("min_step", min_step, 0.005);
    pnh.param<double>("wheel_diameter", wheel_diameter, 0.314);
    ROS_INFO("max_steering_omega: %f", max_steering_omega);
    ROS_INFO("twist_period: %f", twist_period);
    ROS_INFO("body_point_name: %s", body_point_name.c_str());
    ROS_INFO_STREAM("deltav: " << deltav.transpose());
    ROS_INFO("tolerance: %f", tolerance);
    ROS_INFO("min_step: %f", min_step);
    ROS_INFO("wheel_diameter: %f", wheel_diameter);
    ROS_INFO("Waiting for tf");
    lookupPodPosition(std::string("port_suspension"), port_position);
    lookupPodPosition(std::string("starboard_suspension"), starboard_position);
    lookupPodPosition(std::string("stern_suspension"), stern_position);
    ROS_INFO_STREAM("port: [" << port_position.transpose() << "] starboard: [" << starboard_position.transpose() << "] stern: [" << stern_position.transpose() << "]");
    ROS_INFO("Subscribing");
    st = nh.subscribe("twist", 1, &VelocityLimiter::handleTwist, this);
    so = nh.subscribe("odometry", 1, &VelocityLimiter::handleOdometry, this);
    sj = nh.subscribe("joint_state", 1, &VelocityLimiter::handleJointState, this);
    ROS_INFO("Advertising");
    limited_twist = nh.advertise<geometry_msgs::Twist>("limited_twist", 1);
    debug = nh.advertise<platform_motion::Centers>("debug", 1);
    ROS_INFO("Init done");
}

int VelocityLimiter::lookupPodPosition(std::string name, Eigen::Vector2d &pos)
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
    pos << pod_tf.getOrigin().x(), pod_tf.getOrigin().y();
    return 0;
}

void VelocityLimiter::handleTwist(const geometry_msgs::Twist::ConstPtr twist)
{
    Eigen::Vector3d vel_in;
    vel_in << twist->linear.x, twist->linear.y, twist->angular.z;

    boost::unique_lock<boost::mutex> angle_lock(angle_mutex);
    Eigen::Vector3d desired_center, present_center, interpolated_center;
    double desired_omega, present_omega, interpolated_omega;
    Eigen::Vector3d vel_out = platform_motion::NumericalLimit(
        stern_position,
        starboard_position,
        port_position,
        max_steering_omega*twist_period,
        min_step,
        tolerance,
        stern_angle, stern_velocity,
        starboard_angle, starboard_velocity,
        port_angle, port_velocity,
        vel_in,
        present_center, &present_omega,
        desired_center, &desired_omega,
        interpolated_center, &interpolated_omega
        );

    geometry_msgs::Twist lt;
    lt.linear.x = vel_out(0);
    lt.linear.y = vel_out(1);
    lt.angular.z = vel_out(2);
    limited_twist.publish(lt);

    platform_motion::Centers c;
    tf::pointEigenToMsg(present_center, c.present_center);
    c.present_omega.data = present_omega;
    tf::pointEigenToMsg(desired_center, c.desired_center);
    c.desired_omega.data = desired_omega;
    tf::pointEigenToMsg(interpolated_center, c.interpolated_center);
    c.interpolated_omega.data = interpolated_omega;
    debug.publish(c);
}

void VelocityLimiter::handleOdometry(const nav_msgs::Odometry::ConstPtr odo)
{
    boost::unique_lock<boost::mutex> velocity_lock(velocity_mutex);

    body_velocity << odo->twist.twist.linear.x, odo->twist.twist.linear.y, odo->twist.twist.angular.z;
}

bool VelocityLimiter::lookupJoint(const sensor_msgs::JointState::ConstPtr joints, std::string jointName, double *angle, double *velocity)
{
    std::string steering_joint = jointName+"_steering_joint";
    std::string wheel_joint = jointName+"_axle";
    int found=0;
    for(std::vector<std::string>::const_iterator name=joints->name.begin();
            name<joints->name.end();
            name++)
    {
        if( *name == steering_joint )
        {
            int idx = std::distance(joints->name.begin(), name);
            *angle=joints->position[idx];
            found += 1;
        }
        if( *name == wheel_joint )
        {
            int idx = std::distance(joints->name.begin(), name);
            *velocity=joints->velocity[idx];
            *velocity *= wheel_diameter/2.0;
            found += 1;
        }
        if(found == 2)
            return true;
    }
    return false;
}

void VelocityLimiter::handleJointState(const sensor_msgs::JointState::ConstPtr joints)
{
    boost::unique_lock<boost::mutex> angle_lock(angle_mutex);
    double angle, velocity;

    if(lookupJoint(joints, "starboard", &angle, &velocity))
    {
        starboard_angle = angle;
        starboard_velocity = velocity;
    }
    if(lookupJoint(joints, "port", &angle, &velocity))
    {
        port_angle = angle;
        port_velocity = velocity;
    }
    if(lookupJoint(joints, "stern", &angle, &velocity))
    {
        stern_angle = angle;
        stern_velocity = velocity;
    }

    ROS_DEBUG_STREAM("angle stern: " << stern_angle << " starboard: " << starboard_angle << " port: " << port_angle);
    ROS_DEBUG_STREAM("velocity stern: " << stern_velocity << " starboard: " << starboard_velocity << " port: " << port_velocity);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VelocitySmoother");

    VelocityLimiter limit;

    ROS_INFO("spin");
    ros::spin();
}
