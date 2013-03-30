#include <string>

#include "odometry/odometry_node.h"

namespace platform_motion {

OdometryNode::OdometryNode() :
    first(true),
    last_starboard_distance(0), last_port_distance(0), last_stern_distance(0),
    starboard_vel_sum(0), port_vel_sum(0), stern_vel_sum(0),
    vel_sum_count(0),
    odom_position(Eigen::Vector2d::Zero()),
    odom_orientation(0)
{
    ros::NodeHandle param_nh("~");
    param_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    param_nh.param<std::string>("child_frame_id", child_frame_id, "/base_link");
    param_nh.param("delta_threshold", delta_threshold, 0.025);
    param_nh.param("min_translation_norm", min_translation_norm, 0.001);
    param_nh.param("unexplainable_jump", unexplainable_jump, 12.0);
    param_nh.param("wheel_diameter", wheel_diameter, 0.314);

    ROS_INFO("delta_threshold: %f", delta_threshold);
    ROS_INFO("unexplainable_jump : %f", unexplainable_jump);
    joint_state_sub = nh_.subscribe("platform_joint_state", 2, &OdometryNode::jointStateCallback,
            this);
    odometry_pub = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
}

void OdometryNode::init()
{
    ROS_INFO("Waiting for transforms");
    ros::Time current = ros::Time(0);
    ros::Duration wait(1.0);
    while( !(
             listener.waitForTransform(child_frame_id, std::string("/port_suspension"), current, wait) &&
             listener.waitForTransform(child_frame_id, std::string("/starboard_suspension"), current, wait) &&
             listener.waitForTransform(child_frame_id, std::string("/stern_suspension"), current, wait)
            ) && ros::ok()
         )
        ROS_INFO("Still Waiting for transforms");
    if( ros::ok() )
        ROS_INFO("Got all transforms");
}

void OdometryNode::fillOdoMsg(nav_msgs::Odometry *odo, ros::Time stamp, bool stopped)
{
    odo->header.frame_id = odom_frame_id;
    odo->child_frame_id = child_frame_id;
    odo->header.stamp = stamp;

    odo->twist.twist.linear.x = odom_velocity[0];
    odo->twist.twist.linear.y = odom_velocity[1];
    odo->twist.twist.linear.z = 0;
    odo->twist.twist.angular.x = 0;
    odo->twist.twist.angular.y = 0;
    odo->twist.twist.angular.z = odom_omega;

    if(stopped) {
        // covariance is tiny for stopped
        // xx
        odo->pose.covariance[0] = 1e-10;
        // yy
        odo->pose.covariance[7] = 1e-10;
        // xy
        odo->pose.covariance[1] = odo->pose.covariance[6] = 1e-10;
        // xyaw
        odo->pose.covariance[5] = odo->pose.covariance[30] = 1e-10;
        // yyaw
        odo->pose.covariance[11] = odo->pose.covariance[31] = 1e-10;
        // yaw*yaw
        odo->pose.covariance[35] = 1e-10;
        // vxvx
        odo->twist.covariance[0] = 1e-10;
        // vyvy
        odo->twist.covariance[7] = 1e-10;
        // vxvy
        odo->twist.covariance[1] = odo->pose.covariance[6] = 1e-10;
        // vxomegaz
        odo->twist.covariance[5] = odo->pose.covariance[30] = 1e-10;
        // vyomegaz
        odo->twist.covariance[11] = odo->pose.covariance[31] = 1e-10;
        // omegaz*omegaz
        odo->twist.covariance[35] = 1e-10;
    } else {
        // xx
        odo->pose.covariance[0] = 0.05;
        // yy
        odo->pose.covariance[7] = 0.05;
        // xy
        odo->pose.covariance[1] = odo->pose.covariance[6] = 0.005;
        // xyaw
        odo->pose.covariance[5] = odo->pose.covariance[30] = 0.001;
        // yyaw
        odo->pose.covariance[11] = odo->pose.covariance[31] = 0.001;
        // yaw*yaw
        odo->pose.covariance[35] =
            pow((2.*M_PI/((double)10000)), 2);
        // vxvx
        odo->twist.covariance[0] = 10.0;
        // vyvy
        odo->twist.covariance[7] = 10.0;
        // vxvy
        odo->twist.covariance[1] = odo->pose.covariance[6] = 10.0;
        // vxomegaz
        odo->twist.covariance[5] = odo->pose.covariance[30] = 10.0;
        // vyomegaz
        odo->twist.covariance[11] = odo->pose.covariance[31] = 10.0;
        // omegaz*omegaz
        odo->twist.covariance[35] = 10.0;
    }

    // these directions are unmeasured
    odo->pose.covariance[14] = DBL_MAX;
    odo->pose.covariance[21] = DBL_MAX;
    odo->pose.covariance[28] = DBL_MAX;
    odo->twist.covariance[14] = DBL_MAX;
    odo->twist.covariance[21] = DBL_MAX;
    odo->twist.covariance[28] = DBL_MAX;

    // fill out current position
    odo->pose.pose.position.x = odom_position(0);
    odo->pose.pose.position.y = odom_position(1);
    odo->pose.pose.position.z = 0;
    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(odom_orientation);
    odo->pose.pose.orientation = odom_quat;
}

bool OdometryNode::lookupJointValue(const sensor_msgs::JointState::ConstPtr &joint_state,
        std::string requested_name,
        double *position,
        double *velocity)
{
    for(std::vector<std::string>::const_iterator name=joint_state->name.begin();
            name<joint_state->name.end();
            name++) {
        if( *name == requested_name) {
            int idx = std::distance(joint_state->name.begin(), name);
            *position = joint_state->position[idx];
            *velocity = joint_state->velocity[idx];
            //ROS_DEBUG("found %s at %d: %f, %f", requested_name.c_str(), idx, *position, *velocity);
            return true;
        }
    }
    ROS_ERROR("No joint %s in joint_state", requested_name.c_str());
    return false;
}

void OdometryNode::fillMeasurements(const sensor_msgs::JointState::ConstPtr joint_state, struct odometry_measurements *data)
{
    double port_steering, starboard_steering, stern_steering;
    double port_steering_velocity, starboard_steering_velocity, stern_steering_velocity;
    double port_wheel, starboard_wheel, stern_wheel;
    double port_wheel_velocity, starboard_wheel_velocity, stern_wheel_velocity;

    lookupJointValue(joint_state, "port_steering_joint", &port_steering, &port_steering_velocity);
    lookupJointValue(joint_state, "starboard_steering_joint", &starboard_steering, &starboard_steering_velocity);
    lookupJointValue(joint_state, "stern_steering_joint", &stern_steering, &stern_steering_velocity);
    lookupJointValue(joint_state, "port_axle", &port_wheel, &port_wheel_velocity);
    lookupJointValue(joint_state, "starboard_axle", &starboard_wheel, &starboard_wheel_velocity);
    lookupJointValue(joint_state, "stern_axle", &stern_wheel, &stern_wheel_velocity);

    data->port_distance = port_wheel*wheel_diameter/2.;
    data->starboard_distance = starboard_wheel*wheel_diameter/2.;
    data->stern_distance = stern_wheel*wheel_diameter/2.;

    data->port_delta = (data->port_distance - last_port_distance);
    data->starboard_delta = (data->starboard_distance - last_starboard_distance);
    data->stern_delta = (data->stern_distance - last_stern_distance);
    //ROS_DEBUG( "delta: (%6.4f, %6.4f, %6.4f)", port_delta, starboard_delta, stern_delta);
    //ROS_DEBUG( "vel: (%6.4f, %6.4f, %6.4f)", port_wheel_velocity, starboard_wheel_velocity, stern_wheel_velocity);

    starboard_vel_sum += starboard_wheel_velocity*wheel_diameter/2;
    port_vel_sum += port_wheel_velocity*wheel_diameter/2;
    stern_vel_sum += stern_wheel_velocity*wheel_diameter/2;
    vel_sum_count += 1;

    data->port_vel = port_vel_sum/vel_sum_count;
    data->starboard_vel = starboard_vel_sum/vel_sum_count;
    data->stern_vel = stern_vel_sum/vel_sum_count;

    data->port_dir = Eigen::Vector2d(cos(port_steering), sin(port_steering));
    data->stern_dir = Eigen::Vector2d(cos(stern_steering), sin(stern_steering));
    data->starboard_dir = Eigen::Vector2d(cos(starboard_steering), sin(starboard_steering));

    ros::Time current(0);
    tf::StampedTransform port_tf, starboard_tf, stern_tf;

    data->body_pt = Eigen::Vector2d(0,0);

    try {
        listener.lookupTransform(child_frame_id, "/port_suspension", current, port_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up port_suspension: %s", ex.what());
        return;
    }
    data->port_pos = Eigen::Vector2d(port_tf.getOrigin().x(), port_tf.getOrigin().y());

    try {
        listener.lookupTransform(child_frame_id, "/starboard_suspension", current, starboard_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up starboard_suspension: %s", ex.what());
        return;
    }
    data->starboard_pos = Eigen::Vector2d(starboard_tf.getOrigin().x(), starboard_tf.getOrigin().y());

    try {
        listener.lookupTransform(child_frame_id, "/stern_suspension", current, stern_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up stern_suspension: %s", ex.what());
        return;
    }
    data->stern_pos = Eigen::Vector2d(stern_tf.getOrigin().x(), stern_tf.getOrigin().y());

    data->interval = (joint_state->header.stamp - last_joint_message).toSec();

    data->min_translation_norm = min_translation_norm;
}

bool OdometryNode::detectJump(const struct odometry_measurements &data, ros::Time stamp)
{
    double min_abs_delta = std::min(fabs(data.starboard_delta), fabs(data.port_delta));
    min_abs_delta = std::min(min_abs_delta, fabs(data.stern_delta));

    bool jump=min_abs_delta>unexplainable_jump;
    if(jump) {
        ROS_ERROR("Large odometry jump, resetting deltas: %f", min_abs_delta);
    }

    if(first || jump) {
        last_port_distance = data.port_distance;
        last_starboard_distance = data.starboard_distance;
        last_stern_distance = data.stern_distance;
        port_vel_sum = starboard_vel_sum = stern_vel_sum = 0;
        vel_sum_count = 0;
        last_joint_message = stamp;
        ROS_INFO("set initial wheel distance: %f %f %f", last_port_distance, last_starboard_distance, last_stern_distance);
        first = false;
        return true;
    }
    return false;
}

void OdometryNode::resetReferencePose(const struct odometry_measurements &data, const ros::Time &stamp)
{
    starboard_vel_sum = port_vel_sum = stern_vel_sum = 0;
    vel_sum_count = 0;

    last_port_distance = data.port_distance;
    last_starboard_distance = data.starboard_distance;
    last_stern_distance = data.stern_distance;
    last_joint_message = stamp;
}

bool OdometryNode::isMoving(const struct odometry_measurements &data)
{
    double max_abs_delta = std::max(fabs(data.starboard_delta), fabs(data.port_delta));
    max_abs_delta = std::max(max_abs_delta, fabs(data.stern_delta));

    return max_abs_delta>delta_threshold;
}

void OdometryNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr joint_state)
{
    struct odometry_measurements data;

    fillMeasurements(joint_state, &data);

    if(detectJump(data, joint_state->header.stamp))
        return;

    computeOdometry(data, joint_state->header.stamp);

    nav_msgs::Odometry odo;
    fillOdoMsg(&odo, joint_state->header.stamp, isMoving(data));
    odometry_pub.publish(odo);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = joint_state->header.stamp;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = child_frame_id;
    odom_trans.transform.translation.x = odom_position(0);
    odom_trans.transform.translation.y = odom_position(1);
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_orientation);
    odom_broadcaster.sendTransform(odom_trans);
}

}
