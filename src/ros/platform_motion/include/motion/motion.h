namespace platform_motion{

class Motion : public CANOpen::TransferCallbackReceiver {
    public:
        Motion();
        ~Motion();
        void start(void);
        void runBus(void);
        bool ready(void);
        void enable(bool state = true);

    private:
        void twistCallback(const geometry_msgs::Twist::ConstPtr twist);
        void doHome(void);
        void homeComplete(CANOpen::DS301 &node);
        void doEnable(void);
        void enableStateChange(CANOpen::DS301 &node);
        void pvCallback(CANOpen::DS301 &node);
        void syncCallback(CANOpen::SYNC &sync);

        bool intersectLines(Eigen::Vector2d d0, Eigen::Vector2d p0,
                            Eigen::Vector2d d1, Eigen::Vector2d p1,
                            Eigen::Vector2d &pi);

        ros::NodeHandle nh_;

        ros::Subscriber twist_sub;
        ros::Publisher odometry_pub;
        tf::TransformBroadcaster odom_broadcaster;

        actionlib::SimpleActionServer<HomeWheelPodsAction> home_action_server;
        HomeWheelPodsFeedback home_feedback;
        HomeWheelPodsResult home_result;
        int home_count;
        actionlib::SimpleActionServer<EnableWheelPodsAction> enable_action_server;
        EnableWheelPodsFeedback enable_feedback;
        EnableWheelPodsResult enable_result;
        int enable_count;

        std::tr1::shared_ptr<WheelPod> port, starboard, stern;
        std::tr1::shared_ptr<CANOpen::CopleyServo> carousel;

        int port_steering_id, port_wheel_id;
        double port_steering_min, port_steering_max, port_steering_offset;
        int starboard_steering_id, starboard_wheel_id;
        double starboard_steering_min, starboard_steering_max, starboard_steering_offset;
        int stern_steering_id, stern_wheel_id;
        double stern_steering_min, stern_steering_max, stern_steering_offset;
        int carousel_id;
        int sync_interval;

        double wheel_diameter;

        double width, length;
        double center_pt_x, center_pt_y;

        int steering_encoder_counts, wheel_encoder_counts;
        int large_steering_move;

        int CAN_channel, CAN_baud;
        int CAN_fd;
        std::tr1::shared_ptr<CANOpen::Bus> pbus;
        boost::thread CAN_thread;
        bool CAN_thread_run;
        int notify_read_fd, notify_write_fd;

        bool enabled;

        Eigen::Vector2d starboard_pos, port_pos, stern_pos;
        Eigen::Vector2d body_pt;
        double v_stern, v_starboard, v_port;
        double angle_stern, angle_starboard, angle_port;
        double last_starboard_wheel, last_port_wheel, last_stern_wheel;
        int odom_count, odom_counter;
        double port_vel_sum, starboard_vel_sum, stern_vel_sum;
        int pv_counter;
        Eigen::Vector2d odom_position;
        double odom_orientation;
        int odom_frame_id;
};

}


