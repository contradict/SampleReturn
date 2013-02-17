namespace platform_motion{

class Motion : public CANOpen::TransferCallbackReceiver {
    public:
        Motion();
        ~Motion();
        void start(void);
        void shutdown(void);
        void runBus(void);
        bool ready(void);
        void enable_pods(bool state = true);
        void enable_carousel(bool state = true);

    private:
        bool openBus(void);
        void createServos(void);

        void twistCallback(const geometry_msgs::Twist::ConstPtr twist);
        int computePod(Eigen::Vector2d body_vel, double body_omega, Eigen::Vector2d body_pt,
                const char *joint_name, double *steering, double *speed);
        void carouselCallback(const std_msgs::Float64::ConstPtr fmsg);
        void gpioSubscriptionCallback(const platform_motion::GPIO::ConstPtr gpio);
        void doHomePods(void);
        void doHomeCarousel(void);
        void homePodsComplete(CANOpen::DS301 &node);
        void homeCarouselComplete(CANOpen::DS301 &node);
        bool enableWheelPodsCallback(platform_motion::Enable::Request &req,
                                     platform_motion::Enable::Response &resp);
        bool enableCarouselCallback(platform_motion::Enable::Request &req,
                                    platform_motion::Enable::Response &resp);
        void podEnableStateChange(CANOpen::DS301 &node);
        void carouselEnableStateChange(CANOpen::DS301 &node);
        void pvCallback(CANOpen::DS301 &node);
        void gpioCallback(CANOpen::CopleyServo &svo, uint16_t old_pins, uint16_t new_pins);
        void syncCallback(CANOpen::SYNC &sync);
        void reconfigureCallback(PlatformParametersConfig &config, uint32_t level);

        void statusPublishCallback(const ros::TimerEvent& event);

        ros::NodeHandle nh_;
        ros::NodeHandle param_nh;


        tf::TransformListener listener;
        std::string child_frame_id;

        ros::Subscriber twist_sub;
        ros::Subscriber carousel_sub;
        ros::Subscriber gpio_sub;
        ros::Publisher gpio_pub;
        ros::Publisher joint_state_pub;
        ros::Publisher battery_voltage_pub;

        ros::Timer status_publish_timer;

        dynamic_reconfigure::Server<PlatformParametersConfig>
            reconfigure_server;

        actionlib::SimpleActionServer<HomeAction> home_pods_action_server;
        HomeFeedback home_pods_feedback;
        HomeResult home_pods_result;
        int home_pods_count;

        actionlib::SimpleActionServer<HomeAction> home_carousel_action_server;
        HomeFeedback home_carousel_feedback;
        HomeResult home_carousel_result;

        ros::ServiceServer enable_pods_server;
        int enable_pods_count;
        boost::mutex enable_pods_mutex;
        boost::condition_variable enable_pods_cond;

        ros::ServiceServer enable_carousel_server;
        boost::mutex enable_carousel_mutex;
        boost::condition_variable enable_carousel_cond;
 
        std::tr1::shared_ptr<WheelPod> port, starboard, stern;
        std::tr1::shared_ptr<CANOpen::CopleyServo> carousel;

        double wheel_diameter;

        double center_pt_x, center_pt_y;
        double min_wheel_speed;

        int carousel_encoder_counts;
        double carousel_jerk_limit;
        double carousel_profile_velocity;
        double carousel_profile_acceleration;
        double carousel_offset, desired_carousel_position;

        int CAN_fd;
        boost::mutex CAN_mutex;
        std::tr1::shared_ptr<CANOpen::Bus> pbus;
        boost::thread CAN_thread;
        bool CAN_thread_run;

        bool gpio_enabled;
        bool carousel_setup;

        int enable_wait_timeout;
        bool pods_enabled;
        bool desired_pod_state;
        bool carousel_enabled;
        bool desired_carousel_state;

        Eigen::Vector2d body_pt;
        int pv_counter;
        int joint_seq;
};

}


