#include <iostream>

#include "odometry/LLS_odometry.h"

namespace platform_motion
{

LLSOdometryNode::LLSOdometryNode(void):OdometryNode()
{
    ros::NodeHandle param_nh("~");
    param_nh.param("velocity_filter", velocity_filter_, 0.5);
}

void LLSOdometryNode::init(void)
{
    OdometryNode::init();
}

void LLSOdometryNode::computeOdometry(struct odometry_measurements &data, const ros::Time &stamp)
{
    Eigen::Matrix<double, 6, 1> measured_motions;
    measured_motions.block<2,1>(0,0) = data.port_delta*data.port_dir;
    measured_motions.block<2,1>(2,0) = data.starboard_delta*data.starboard_dir;
    measured_motions.block<2,1>(4,0) = data.stern_delta*data.stern_dir;

    Eigen::Matrix<double, 6, 1> measured_velocities;
    measured_velocities.block<2,1>(0,0) = data.port_vel*data.port_dir;
    measured_velocities.block<2,1>(2,0) = data.starboard_vel*data.starboard_dir;
    measured_velocities.block<2,1>(4,0) = data.stern_vel*data.stern_dir;

    //(dtheta X R) + dx = dwheel;
    // dtheta*[-Ry, Rx] + [dx, dy] = delta*[cos(steer), sin(steer)]
    Eigen::Matrix2d cross;
    cross << 0, -1,
             1,  0;
    Eigen::Matrix<double, 6, 3> shape;
    Eigen::Vector2d R;
    R = (data.port_pos - data.body_pt);
    shape.block<2,1>(0,0) = cross * R;
    shape.block<2,2>(0,1) = Eigen::Matrix2d::Identity();
    R = (data.starboard_pos - data.body_pt);
    shape.block<2,1>(2,0) = cross * R;
    shape.block<2,2>(2,1) = Eigen::Matrix2d::Identity();
    R = (data.stern_pos - data.body_pt);
    shape.block<2,1>(4,0) = cross * R;
    shape.block<2,2>(4,1) = Eigen::Matrix2d::Identity();

    Eigen::Matrix<double, 3, 6> solution = (shape.transpose()*shape).inverse()*shape.transpose();
    Eigen::Vector3d motion = solution*measured_motions;
    Eigen::Vector3d velocity = solution*measured_velocities;

    // integrate in odometry frame
    Eigen::Rotation2Dd rotation(odom_orientation);
    Eigen::Vector2d OdometryFrameVelocity = rotation*motion.segment<2>(1);
    odom_position += OdometryFrameVelocity;
    odom_orientation += motion(0);
    while(odom_orientation > 2*M_PI) odom_orientation -= 2*M_PI;
    while(odom_orientation < 0) odom_orientation += 2*M_PI;
    odom_velocity = velocity_filter_*motion.segment<2>(1)/data.interval + (1.0-velocity_filter_)*velocity.segment<2>(1);
    odom_omega = velocity_filter_*motion(0)/data.interval + (1.0-velocity_filter_)*velocity(0);
    //ROS_DEBUG_STREAM("\n\tdx/dt=" << motion.segment<2>(0).transpose()/data.interval << "\n\tvel=" << velocity.segment<2>(0).transpose());

    resetReferencePose(data, stamp);

}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");

    platform_motion::LLSOdometryNode on;

    on.init();

    ros::spin();

    return 0;
}
