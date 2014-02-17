#include <math.h>
#include <vector>

#include <Eigen/Dense>
#include <CANOpen.h>

#include <motion/wheelpod.h>

namespace platform_motion {

void WheelPod::setCallbacks(CANOpen::DS301CallbackObject wheelcb,
      CANOpen::DS301CallbackObject steeringcb,
      CANOpen::CopleyServo::InputChangeCallback gpiocb,
      CANOpen::DS301CallbackObject wheelBufferCallback,
      CANOpen::DS301CallbackObject steeringBufferCallback
    )
{
    wheel.setPVCallback(wheelcb);
    wheel.setInputCallback(gpiocb);
    steering.setPVCallback(steeringcb);
    steering.setInputCallback(gpiocb);

    wheel.setMoreDataNeededCallback(wheelBufferCallback);
    steering.setMoreDataNeededCallback(steeringBufferCallback);
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

void WheelPod::move(PodSegment &segment)
{
    // we're going to use pvt mode here
    // _setMode won't do anything if the mode is already PodPosition,
    // so there's not much harm in callign it repeatedly
    _setMode(PodPosition);

    // save the current buffer depths so we know to tell the servo to start
    // going if we've just transitioned from a buffer underflow.
    int steeringBufferDepth = steering.getPvtBufferDepth();
    int wheelBufferDepth = wheel.getPvtBufferDepth();

    // variables for the unit conversions:
    int steeringPosition, steeringVelocityCounts;
    int wheelDistanceCounts, wheelVelocityCounts;

    // check the segment duration. the servos can only handle 8-bit
    // time in milliseconds per segment
    if(segment.duration <= 0.255)
    {
        int time = round(segment.duration*1000.0);
        // handle potential rounding problems.
        if(time > 255)
        {
            time = 255;
        }
        // convert units to counts with the helper.
        computeSteeringAndVelocity(
                segment.steeringAngle,
                segment.steeringVelocity,
                segment.wheelDistance,
                segment.wheelVelocity,
                steeringPosition,
                steeringVelocityCounts,
                wheelDistanceCounts,
                wheelVelocityCounts
        );

        // send the segment to the servos
        wheel.addPvtSegment(
                wheelDistanceCounts,
                wheelVelocityCounts,
                time
        );
        steering.addPvtSegment(
                steeringPosition,
                steeringVelocityCounts,
                time
        );
    }
    else
    {
        // you probably don't want to be here.
        // linearly interpolate the segment so the time works out.
        // this is likely wrong most of the time, but is better
        // than telling the servo to do something totally crazy.
        int numSubsegments = (int)ceil(segment.duration/0.255);

        double subsegmentDuration = segment.duration/numSubsegments;

        // compute deltas between this state and the last.
        double steeringAngleDelta = 
            (segment.steeringAngle - m_lastSegment.steeringAngle)/
            numSubsegments;
        double steeringVelocityDelta = 
            (segment.steeringVelocity - m_lastSegment.steeringVelocity)/
            numSubsegments;
        double wheelVelocityDelta = 
            (segment.wheelVelocity - m_lastSegment.wheelVelocity)/
            numSubsegments;

        // add the subsegments
        for(int i = 1; i <= numSubsegments; i++)
        {
            int time = round(subsegmentDuration*1000.0);
            // handle potential rounding problems.
            if(time > 255)
            {
                time = 255;
            }
            // convert units to counts with the helper.
            computeSteeringAndVelocity(
                m_lastSegment.steeringAngle + i*steeringAngleDelta,
                m_lastSegment.steeringVelocity + i*steeringVelocityDelta,
                segment.wheelDistance/numSubsegments,
                m_lastSegment.wheelVelocity + i*wheelVelocityDelta,
                steeringPosition,
                steeringVelocityCounts,
                wheelDistanceCounts,
                wheelVelocityCounts
            );

            // send the segment to the servos
            wheel.addPvtSegment(
                    wheelDistanceCounts,
                    wheelVelocityCounts,
                    time
            );
            steering.addPvtSegment(
                    steeringPosition,
                    steeringVelocityCounts,
                    time
            );
        }
    }

    // save the last segment. it may be useful.
    m_lastSegment = segment;

    // start the servos moving as soon as they've got enough data.
    // the mode change already flips this bit, so it may be redundant to do
    // it here. if things are starnge and jittery, consider removing these
    // calls to startPvtMove.
    if((steeringBufferDepth < 2) && (steering.getPvtBufferDepth() >= 2))
    {
        steering.startPvtMove();
    }
    if((wheelBufferDepth < 2) && (wheel.getPvtBufferDepth() >= 2))
    {
        wheel.startPvtMove();
    }
}

void WheelPod::move(std::vector<PodSegment> &segments)
{
    // loop over the segments and use the one segment move method to
    // digest all the given segments
    for(auto segment : segments)
    {
        move(segment);
    }
}

void WheelPod::computeSteeringAndVelocity(
    double steeringAngle,
    double steeringVelocity,
    double wheelDistance,
    double wheelVelocity,
    int &steeringPosition,
    int &steeringVelocityCounts,
    int &wheelDistanceCounts,
    int &wheelVelocityCounts
    )
{
    // account for the steering angle offset
    steeringAngle -= steering_offset;
    // keep steering inside of acceptable range
    if(steeringAngle > steering_max)
    {
        steeringAngle -= M_PI;
        steeringVelocity *= -1.0;
        wheelVelocity *= -1.0;
        wheelDistance *= -1.0;
    }
    if(steeringAngle < steering_min)
    {
        steeringAngle += M_PI;
        steeringVelocity *= -1.0;
        wheelVelocity *= -1.0;
        wheelDistance *= -1.0;
    }

    steeringPosition = round(steering_encoder_counts*steeringAngle/2.0/M_PI);
    steeringVelocityCounts =
        round(steering_encoder_counts*10.0*steeringVelocity/2.0/M_PI);

    wheelDistanceCounts = 
        round(wheel_encoder_counts * (wheelDistance/(wheelDiameter*M_PI)));
    wheelVelocityCounts =
        round(wheel_encoder_counts*10.0 * (wheelVelocity/(wheelDiameter*M_PI)));
}

void WheelPod::_setMode(enum PodMode m)
{
    if(currentMode == m) return;
    switch(m) {
        case PodPosition:
            steering.mode(CANOpen::InterpolatedPosition);
            steering.setPvtAbsolute();
            wheel.mode(CANOpen::InterpolatedPosition);
            wheel.setPvtRelative();
            // flip the bit that will cause pvt segments to start when they're added
            steering.startPvtMove();
            wheel.startPvtMove();
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

void WheelPod::getStatusWord(uint16_t *steering_status, uint16_t *wheel_status)
{
    *steering_status = steering.getStatusWord();
    *wheel_status = wheel.getStatusWord();
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

int WheelPod::getMinBufferDepth()
{
    return std::min(wheel.getPvtBufferDepth(), steering.getPvtBufferDepth());
}

}
