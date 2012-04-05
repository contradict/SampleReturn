#include <math.h>

#include <Eigen/Dense>
#include <CANOpen.h>

#include <motion/wheelpod.h>

namespace platform_motion {

void WheelPod::drive(double angle, double omega)
{
    _setMode(PodDrive);

    double desired_steering_position = angle-steering_offset;
    if(desired_steering_position>steering_max) {
        desired_steering_position -= M_PI;
        omega *= -1.0;
    }
    if(desired_steering_position<steering_min) {
        desired_steering_position += M_PI;
        omega *= -1.0;
    }
    int position = round(steering_encoder_counts*desired_steering_position/2./M_PI);
    velocity = round(wheel_encoder_counts*10.0*omega/2./M_PI);
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

void WheelPod::enable(void)
{
    steering.enable();
    wheel.enable();
}

void WheelPod::initialize(void)
{
    steering.initialize();
    wheel.initialize();
}

}
