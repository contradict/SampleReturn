#include <string>

#include "odometry/odometry_node.h"

namespace platform_motion {

std::ostream & operator<< (std::ostream &out, const odometry_measurements &m)
{
    out << "port: (" << m.port_pos.transpose() << ") (" << m.port_dir.transpose() << ") " << m.port_distance << " " << m.port_delta << " " << m.port_vel << " " << m.port_angle << std::endl;
    out << "stern: (" << m.stern_pos.transpose() << ") (" << m.stern_dir.transpose() << ") " << m.stern_distance << " " << m.stern_delta << " " << m.stern_vel << " " << m.stern_angle << std::endl;
    out << "starboard: (" << m.starboard_pos.transpose() << ") (" << m.starboard_dir.transpose() << ") " << m.starboard_distance << " " << m.starboard_delta << " " << m.starboard_vel << " " << m.starboard_angle << std::endl;
    return out;
}


OdometryNode::OdometryNode() :
    publish_tf(true),
    first(true),
    last_starboard_distance(0), last_port_distance(0), last_stern_distance(0),
    starboard_vel_sum(0), port_vel_sum(0), stern_vel_sum(0),
    vel_sum_count(0),
    odom_position(Eigen::Vector2d::Zero()),
    odom_orientation(0),
    odom_pose_covariance(Eigen::VectorXd::Zero(36)),
    odom_twist_covariance(Eigen::VectorXd::Zero(36)),
    odom_twist_stopped_covariance(Eigen::VectorXd::Zero(36))
{
    ros::NodeHandle param_nh("~");
    param_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    param_nh.param<std::string>("child_frame_id", child_frame_id, "/base_link");
    param_nh.param("delta_threshold", delta_threshold, 0.025);
    param_nh.param("min_translation_norm", min_translation_norm, 0.001);
    param_nh.param("unexplainable_jump", unexplainable_jump, 12.0);
    param_nh.param("wheel_diameter", wheel_diameter, 0.314);
    param_nh.param("publish_tf", publish_tf, true);

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
    getPodPositions(&data);
    fillDefaultCovariance();
}

void OdometryNode::fillDefaultCovariance(void)
{
    /***************************
     *   POSE COVARIANCE
     ***************************/
    // xx
    odom_pose_covariance[0] = 0.05;
    // yy
    odom_pose_covariance[7] = 0.05;
    // xy
    odom_pose_covariance[1] = odom_pose_covariance[6] = 0.005;
    // xyaw
    odom_pose_covariance[5] = odom_pose_covariance[30] = 0.001;
    // yyaw
    odom_pose_covariance[11] = odom_pose_covariance[31] = 0.001;
    // yaw*yaw
    odom_pose_covariance[35] =
        pow((2.*M_PI/((double)10000)), 2);
    // these directions are unmeasured
    odom_pose_covariance[14] = DBL_MAX;
    odom_pose_covariance[21] = DBL_MAX;
    odom_pose_covariance[28] = DBL_MAX;

    /***************************
     *   TWIST COVARIANCE
     *     (in motion)
     ***************************/
    // vxvx
    odom_twist_covariance[0] = 10.0;
    // vyvy
    odom_twist_covariance[7] = 10.0;
    // vxvy
    odom_twist_covariance[1] = odom_twist_covariance[6] = 10.0;
    // vxomegaz
    odom_twist_covariance[5] = odom_twist_covariance[30] = 10.0;
    // vyomegaz
    odom_twist_covariance[11] = odom_twist_covariance[31] = 10.0;
    // omegaz*omegaz
    odom_twist_covariance[35] = 10.0;
    // unmeasured
    odom_twist_covariance[14] = DBL_MAX;
    odom_twist_covariance[21] = DBL_MAX;
    odom_twist_covariance[28] = DBL_MAX;

    /***************************
     *   TWIST COVARIANCE
     *      (stopped)
     ***************************/
    // vxvx
    odom_twist_stopped_covariance[0] = 1e-10;
    // vyvy
    odom_twist_stopped_covariance[7] = 1e-10;
    // vxvy
    odom_twist_stopped_covariance[1] = odom_twist_stopped_covariance[6] = 1e-10;
    // vxomegaz
    odom_twist_stopped_covariance[5] = odom_twist_stopped_covariance[30] = 1e-10;
    // vyomegaz
    odom_twist_stopped_covariance[11] = odom_twist_stopped_covariance[31] = 1e-10;
    // omegaz*omegaz
    odom_twist_stopped_covariance[35] = 1e-10;
    // unmeasured
    odom_twist_stopped_covariance[14] = DBL_MAX;
    odom_twist_stopped_covariance[21] = DBL_MAX;
    odom_twist_stopped_covariance[28] = DBL_MAX;
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

    for(int i=0;i<36;i++)
    {
        odo->pose.covariance[i] = odom_pose_covariance[i];
        if(stopped) {
            odo->twist.covariance[i] = odom_twist_stopped_covariance[i];
        } else {
            odo->twist.covariance[i] = odom_twist_covariance[i];
        }
    }

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
    data->port_angle = port_steering;
    data->stern_dir = Eigen::Vector2d(cos(stern_steering), sin(stern_steering));
    data->stern_angle = stern_steering;
    data->starboard_dir = Eigen::Vector2d(cos(starboard_steering), sin(starboard_steering));
    data->starboard_angle = starboard_steering;

    data->body_pt = Eigen::Vector2d(0,0);

    data->interval = (joint_state->header.stamp - last_joint_message).toSec();

    data->min_translation_norm = min_translation_norm;
}

void OdometryNode::getPodPositions(struct odometry_measurements *data)
{
    ros::Time current(0);
    tf::StampedTransform port_tf, starboard_tf, stern_tf;

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
    fillMeasurements(joint_state, &data);

    if(detectJump(data, joint_state->header.stamp))
        return;

    computeOdometry(data, joint_state->header.stamp);

    nav_msgs::Odometry odo;
    fillOdoMsg(&odo, joint_state->header.stamp, isMoving(data));
    odometry_pub.publish(odo);

    if(publish_tf)
    {
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

}
