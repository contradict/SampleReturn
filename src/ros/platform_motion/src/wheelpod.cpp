#include <math.h>

#include <Eigen/Dense>
#include <CANOpen.h>

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
