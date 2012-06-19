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
                 std::string steering_joint_name,
                 long int steering_id,
                 double steering_min,
                 double steering_max,
                 double steering_offset,
                 long int steering_encoder_counts,
                 std::string wheel_joint_name,
                 long int wheel_id,
                 long int wheel_encoder_counts,
                 double wheel_diameter,
                 double min_wheel_speed
                 ):
            steering(CANOpen::CopleyServo(steering_id, pbus)),
            wheel(CANOpen::CopleyServo(wheel_id, pbus)),
            wheel_diameter(wheel_diameter),
            steering_joint_name(steering_joint_name),
            steering_min(steering_min),
            steering_max(steering_max),
            steering_offset(steering_offset),
            steering_encoder_counts(steering_encoder_counts),
            wheel_joint_name(wheel_joint_name),
            wheel_encoder_counts(wheel_encoder_counts),
            min_wheel_speed(min_wheel_speed),
            currentMode(PodUninitialized)
            {};

        void setCallbacks(CANOpen::DS301CallbackObject wheelcb,
                          CANOpen::DS301CallbackObject steeringcb,
                          CANOpen::CopleyServo::InputChangeCallback gpiocb);

        void initialize(void);

        void drive(double angle, double omega);
        int driveBody(Eigen::Vector2d body_vel, double body_omega, std::string body_frame,
                double *steering_angle, double *wheel_speed);
        void move(Eigen::Vector2d distance, double rotation);
        void home(CANOpen::DS301CallbackObject cb);
        void enable(bool state, CANOpen::DS301CallbackObject cb);
        bool enabled(void);
        bool ready(void);

        void getPosition(double *steering_pos, double *steering_vel, double *wheel_pos, double *wheel_vel);

        void setSteeringOffset(double offset);

        CANOpen::CopleyServo steering, wheel;

        double wheel_diameter;
    private:
        int computePodVelocity(Eigen::Vector2d body_vel, double body_omega, std::string body_frame,
                double *steering, double *speed);
        void _setMode(enum PodMode m);
        void _positionAcchieved(CANOpen::DS301 &node);

        tf::TransformListener listener;

        std::string steering_joint_name;
        double steering_min, steering_max, steering_offset;
        long int steering_encoder_counts;
        std::string wheel_joint_name;
        long int wheel_encoder_counts;
        double min_wheel_speed;

        double desired_steering_position, desired_omega;

        enum PodMode currentMode;

};

}
