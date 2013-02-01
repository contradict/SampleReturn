#include <math.h>

#include <Eigen/Dense>
#include <CANOpen.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <motion/wheelpod.h>

namespace platform_motion {

void WheelPod::setCallbacks(CANOpen::DS301CallbackObject wheelcb,
        CANOpen::DS301CallbackObject steeringcb,
        CANOpen::CopleyServo::InputChangeCallback gpiocb)
{
    wheel.setPVCallback(wheelcb);
    wheel.setInputCallback(gpiocb);
    steering.setPVCallback(steeringcb);
    steering.setInputCallback(gpiocb);
}

void WheelPod::drive(double angle, double omega)
{
    _setMode(PodDrive);

    desired_steering_position = angle-steering_offset;
    if(desired_steering_position>steering_max) {
        desired_steering_position -= M_PI;
        omega *= -1.0;
    }
    if(desired_steering_position<steering_min) {
        desired_steering_position += M_PI;
        omega *= -1.0;
    }
    desired_omega = omega;
    int position = round(steering_encoder_counts*desired_steering_position/2./M_PI);
    /*
    if(abs(position - steering.position)>large_steering_move) {
        wheel.setVelocity(0);
    }
    */
    steering.setPosition(position,
            CANOpen::DS301CallbackObject(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&WheelPod::_positionAcchieved)));
}

int WheelPod::driveBody(Eigen::Vector2d body_vel, double body_omega, std::string body_frame,
        double *steering, double *speed)
{
    getPosition(steering, NULL, NULL, NULL);
    int err=computePodVelocity(body_vel, body_omega, body_frame, steering, speed);
    return err;
}

int WheelPod::lookupPodPosition(std::string body_frame, tf::Pose *pod_pose)
{
    ros::Time current(0);
    try {
        listener.lookupTransform(steering_joint_name, body_frame, current, *pod_pose );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", steering_joint_name.c_str(), ex.what());
        return -1;
    }
    //ROS_DEBUG("%s at (%f, %f)", joint_name, port_tf.getOrigin().x(), port_tf.getOrigin().y());
    return 0;
}

int WheelPod::computePodVelocity(tf::Vector3 body_vel, double body_omega, std::string body_frame,
        double *steering, double *speed)
{
    tf::Pose pod_pose;
    int err = lookupPodPosition(body_frame, &pod_pose);
    if(err<0)
        return err;
    tf::Vector3 omega_vect(0, 0, body_omega);
    tf::Vector3 pod_vel = body_vel + omega_vect.cross(pod_pose.getOrigin());

    *speed = pod_vel.length();
    if(*speed>min_wheel_speed) {
        *steering = atan2(pod_vel.y(), pod_vel.x());
        *speed /= M_PI*wheel_diameter;
        return 0;
    } else {
        *speed = 0;
        return 1;
    }
}

int WheelPod::bodyToPod( const std::vector<geometry_msgs::PoseStamped> &body_position,
                         const std::vector<geometry_msgs::Twist> &body_velocity,
                         std::string body_frame,
                         std::vector<struct CANOpen::PVTData> *pod_steer,
                         std::vector<struct CANOpen::PVTData> *pod_wheel
                        )
{
    std::vector<geometry_msgs::Pose>::const_iterator p=body_position.begin();
    std::vector<geometry_msgs::Twist>::const_iterator v=body_velocity.begin();
    struct CANOpen::PVTData steer_point, wheel_point;

    tf::Pose pod_pose;
    int err = lookupPodPosition(odom_frame, &position);
    if(err<0)
        return err;

    computePodMove( pod_pose,
                    *p, *v,
                    *p, *v,
                    &steer_point, &wheel_point);


void WheelPod::computePodMove( const tf::Pose &position,
                               const geometry_msgs::PoseStamped &prev_pos,
                               const geometry_msgs::Twist &prev_vel,
                               const geometry_msgs::PoseStamped &pos,
                               const geometry_msgs::Twist &vel,
                               struct CANOpen::PVTData *steer_point,
                               struct CANOpen::PVTData *wheel_point)
{
    tf::Quaternion body_q;
    double body_yaw;
    double steer_angle, steer_velocity;
    double wheel_angle, wheel_velocity;

    // retrieve next desired orientation
    tf::Pose p;
    tf::poseMsgToTF( *pos, p);

    // vector from desired odom position to desired pod position
    tf::Vector3 pod_offset = position.inverse*

    // initialize previous position, velocity to first point
    Eigen::Vector3d ep(p->position.x, p->position.y, 0);
    prev_pod_position = ep + pod_offset;

    Eigen::Vector3d rvec(v->angular.x, v->angular.y, v->angular.z);
    prev_pod_velocity = Eigen::Vector3d(v->linear.x, v->linear.y, v->linear.z) +
                        pod_offset.cross(rvec);

    // if not moving, use relative steering move since no
    // new direction is available
    if(prev_pod_velocity.norm()>0.01) {
        steer_angle = atan2(pod_velocity[1], pod_velocity[0]) - body_yaw;
        steer_point.relative = false;
    } else {
        steer_angle = 0;
        steer_point.relative = true;
    }
    steer_point.position = steering_encoder_counts*steer_angle/2.0/M_PI;
    steer_point.velocity = 

    p++, v++;
    for(/* p, v created above */; p<body_position.end(); p++,v++) {

        tf::quaternionMsgToTF(p->orientation, body_q);
        Eigen::Rotation2D<double> body_rotation(tf::getYaw(body_q));
        pod_offset.head(2) = body_rotation*(position-body_pt);
        pod_offset(2) = 0;
        Eigen::Vector3d ep(p->position.x, p->position.y);
        pod_position = ep+pod_offset;

        Eigen::Vector3d rvec(v->angular.x, v->angular.y, v->angular.z);
        pod_velocity = Eigen::Vector3d(v->linear.x, v->linear.y, v->linear.z) +
                       pod_offset.cross(rvec);

        if(pod_velocity.norm()>0.01) {
            steer_angle = atan2(pod_velocity[1], pod_velocity[0]);
            steer_point.relative = false;
        } else {
            steer_angle = 0;
            steer_point.relative = true;
        }
    }
}



void WheelPod::_positionAcchieved(CANOpen::DS301 &node)
{
    int velocity = round(wheel_encoder_counts*10.0*desired_omega);
    wheel.setVelocity(velocity);
}

void WheelPod::home(CANOpen::DS301CallbackObject cb)
{
    steering.home(cb);
}

void WheelPod::move(Eigen::Vector2d distance, double rotation)
{
    _setMode(PodPosition);
    // TODO: Relative absolute move
}

void WheelPod::_setMode(enum PodMode m)
{
    if(currentMode == m) return;
    switch(m) {
        case PodPosition:
            steering.mode(CANOpen::ProfilePosition);
            wheel.mode(CANOpen::ProfilePosition);
            currentMode = m;
            break;
        case PodDrive:
            steering.mode(CANOpen::ProfilePosition);
            wheel.mode(CANOpen::ProfileVelocity);
            currentMode = m;
            break;
        case PodUninitialized:
            break;
    }
}

bool WheelPod::ready(void)
{
    return steering.ready() && wheel.ready();
}

void WheelPod::enable(bool state, CANOpen::DS301CallbackObject cb)
{
    steering.enable(state, cb);
    wheel.enable(state, cb);
}

bool WheelPod::enabled(void)
{
    return steering.enabled() && wheel.enabled();
}

void WheelPod::initialize(void)
{
    steering.initialize();
    wheel.initialize();
}

void WheelPod::getPosition(double *steering_pos, double *steering_vel, double *wheel_pos, double *wheel_vel)
{
    if(steering_pos)
        *steering_pos =
            ((double)steering.position)/((double)steering_encoder_counts)*2*M_PI +
            steering_offset;
    if(steering_vel)
        *steering_vel =
            ((double)steering.velocity)/((double)steering_encoder_counts)*2*M_PI/10.;

    if(wheel_pos)
        *wheel_pos =
            ((double)wheel.position)/((double)wheel_encoder_counts);
    if(wheel_vel)
        *wheel_vel =
            ((double)wheel.velocity)/((double)wheel_encoder_counts)/10.;

}

void WheelPod::setSteeringOffset(double offset)
{
    double delta = offset - steering_offset;
    steering_offset = offset;
    desired_steering_position-=delta;
    if(desired_steering_position>steering_max) {
        desired_steering_position -= M_PI;
        desired_omega *= -1.0;
    }
    if(desired_steering_position<steering_min) {
        desired_steering_position += M_PI;
        desired_omega *= -1.0;
    }
    if(ready()) {
        int position = round(steering_encoder_counts*desired_steering_position/2./M_PI);
        steering.setPosition(position,
                CANOpen::DS301CallbackObject(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                    static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&WheelPod::_positionAcchieved)));
    }
}

}
