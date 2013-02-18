namespace platform_motion {

enum PodMode {
    PodUninitialized,
    PodDrive,
    PodPosition
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
                 long int large_steering_move
                 ):
            steering(CANOpen::CopleyServo(steering_id, pbus)),
            wheel(CANOpen::CopleyServo(wheel_id, pbus)),
            steering_min(steering_min),
            steering_max(steering_max),
            steering_offset(steering_offset),
            steering_encoder_counts(steering_encoder_counts),
            wheel_encoder_counts(wheel_encoder_counts),
            large_steering_move(large_steering_move),
            currentMode(PodUninitialized)
            {};

        void setCallbacks(CANOpen::DS301CallbackObject wheelcb,
                          CANOpen::DS301CallbackObject steeringcb,
                          CANOpen::CopleyServo::InputChangeCallback gpiocb);

        void initialize(void);

        void drive(double angle, double omega);
        void move(Eigen::Vector2d distance, double rotation);
        void home(CANOpen::DS301CallbackObject cb);
        void enable(bool state, CANOpen::DS301CallbackObject cb);
        bool enabled(void);
        bool ready(void);

        void getPosition(double *steering_pos, double *steering_vel, double *wheel_pos, double *wheel_vel);
        void getStatusWord(uint16_t *steering_status, uint16_t *wheel_status);

        void setSteeringOffset(double offset);

        CANOpen::CopleyServo steering, wheel;

    private:
        void _setMode(enum PodMode m);
        void _positionAcchieved(CANOpen::DS301 &node);

        double steering_min, steering_max, steering_offset;
        long int steering_encoder_counts, wheel_encoder_counts;
        long int large_steering_move;

        double desired_steering_position, desired_omega;

        enum PodMode currentMode;

};

}
