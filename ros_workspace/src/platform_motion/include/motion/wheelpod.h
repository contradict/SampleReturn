namespace platform_motion {

enum PodMode {
    PodUninitialized,
    PodDrive,
    PodPosition
};

struct PodSegment
{
    double steeringAngle; // absolute in radians
    double steeringVelocity; // radians per second
    double wheelDistance; // signed relative distance in meters
    double wheelVelocity; // signed velocity in meters per second
    double duration; // segment length in seconds

    PodSegment():
        steeringAngle(0.0),
        steeringVelocity(0.0),
        wheelDistance(0.0),
        wheelVelocity(0.0),
        duration(0.0)
    {}
};

class WheelPod : public CANOpen::TransferCallbackReceiver {
    public:
        WheelPod(
                 std::tr1::shared_ptr<CANOpen::Bus> pbus,
                 long int steering_id,
                 long int wheel_id,
                 double steering_min,
                 double steering_max,
                 double steering_offset,
                 long int steering_encoder_counts,
                 long int wheel_encoder_counts,
                 long int large_steering_move,
                 double wheel_diameter
                 ):
            steering(CANOpen::CopleyServo(steering_id, pbus)),
            wheel(CANOpen::CopleyServo(wheel_id, pbus)),
            steering_min(steering_min),
            steering_max(steering_max),
            steering_offset(steering_offset),
            steering_encoder_counts(steering_encoder_counts),
            wheel_encoder_counts(wheel_encoder_counts),
            large_steering_move(large_steering_move),
            wheelDiameter(wheel_diameter),
            currentMode(PodUninitialized),
            m_isBackwards(false)
            {};

        // some wheel pods are mounted  backwards. set a flag to let them know!
        void setIsBackwards(bool isBackwards) {m_isBackwards = isBackwards;}

        void setCallbacks(CANOpen::DS301CallbackObject wheelcb,
                          CANOpen::DS301CallbackObject steeringcb,
                          CANOpen::CopleyServo::InputChangeCallback gpiocb,
                          CANOpen::DS301CallbackObject wheelBufferCallback,
                          CANOpen::DS301CallbackObject steeringBufferCallback
            );

        void initialize(void);

        void drive(double angle, double omega);
        void move(std::vector<PodSegment> &segments);
        void move(PodSegment &segment);
        void home(CANOpen::DS301CallbackObject cb);
        void enable(bool state, CANOpen::DS301CallbackObject cb);
        bool enabled(void);
        bool ready(void);

        // for pvt mode we need to coordinate our starting. do we need to
        // start?
        bool getNeedsToStart();
        void startMoving(); // actually start pvt moves.

        void getPosition(double *steering_pos, double *steering_vel, double *wheel_pos, double *wheel_vel);
        void getStatusWord(uint16_t *steering_status, uint16_t *wheel_status);

        void setSteeringOffset(double offset);

        int getMinBufferDepth();

        CANOpen::CopleyServo steering, wheel;

    private:
        void _setMode(enum PodMode m);
        void _positionAcchieved(CANOpen::DS301 &node);

        void computeSteeringAndVelocity(
                double steeringAngle,
                double steeringVelocity,
                double wheelDistance,
                double wheelVelocity,
                int &steeringPosition,
                int &steeringVelocityCounts,
                int &wheelDistanceCounts,
                int &wheelVelocityCounts
            );

        double steering_min, steering_max, steering_offset;
        long int steering_encoder_counts, wheel_encoder_counts;
        long int large_steering_move;

        double desired_steering_position, desired_omega;

        double wheelDiameter;

        enum PodMode currentMode;

        PodSegment m_lastSegment;

        bool m_isBackwards;
};

}
