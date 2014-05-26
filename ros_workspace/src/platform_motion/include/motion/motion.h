#include <mutex>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <platform_motion_msgs/Path.h>

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

        struct BodySegment
        {
            // when a Knot has been transformed into wheel pod local coordinates,
            // it becomes a BodySegment
            PodSegment port;
            PodSegment starboard;
            PodSegment stern;
            ros::Time time;
        };

        bool openBus(void);
        void createServos(void);

        void plannerTwistCallback(const geometry_msgs::Twist::ConstPtr twist);
        void joystickTwistCallback(const geometry_msgs::Twist::ConstPtr twist);
        void servoTwistCallback(const geometry_msgs::Twist::ConstPtr twist);
        void plannedPathCallback(const platform_motion_msgs::Path::ConstPtr path);
        void handleTwist(const geometry_msgs::Twist::ConstPtr twist);
        int computePod(Eigen::Vector2d body_vel, double body_omega, Eigen::Vector2d body_pt,
                Eigen::Vector2d pod_pos, double *steering, double *speed);
        void carouselCallback(const std_msgs::Float64::ConstPtr fmsg);
        void gpioSubscriptionCallback(const platform_motion_msgs::GPIO::ConstPtr gpio);
        void doHomePods(void);
        void doHomeCarousel(void);
        void cancelHome( void );
        void homePodsComplete(CANOpen::DS301 &node);
        void homeCarouselComplete(CANOpen::DS301 &node);
        bool enableCarouselCallback(platform_motion_msgs::Enable::Request &req,
                                    platform_motion_msgs::Enable::Response &resp);
        bool selectMotionModeCallback(platform_motion_msgs::SelectMotionMode::Request &req,
                                     platform_motion_msgs::SelectMotionMode::Response &resp);
        bool handleEnablePods(bool en, bool *result);
        void handlePause( void );
        void handleUnlock( void );
        void driveToUnLock( void );
        void planToZeroTwist( void );
        void driveToLock( void );
        void handleLock( void );
        void waitForLockCond( void );
        void pvtToZero( void );
        void pvtToLock( void );
        void pvtToUnlock( void );
        bool setSteering( PodSegment &pod, double goal, double dt);
        void podEnableStateChange(CANOpen::DS301 &node);
        void carouselEnableStateChange(CANOpen::DS301 &node);
        void pvCallback(CANOpen::DS301 &node);
        void gpioCallback(CANOpen::CopleyServo &svo, uint16_t old_pins, uint16_t new_pins);
        void syncCallback(CANOpen::SYNC &sync);
        void statusCallback(CANOpen::DS301 &node);
        void reconfigureCallback(PlatformParametersConfig &config, uint32_t level);

        void statusPublishCallback(const ros::TimerEvent& event);

        bool targetReached( void );

        // pvt mode
        bool pathToBody( void );
        BodySegment interpolatePodSegments(const BodySegment &first, const BodySegment &second, const BodySegment &last, ros::Time now);
        bool checkSegmentAcceleration();
        double computeSafeInterval(double dt);
        Eigen::Vector2d podVelocity(const platform_motion_msgs::Knot &current, Eigen::Vector3d pod_pos);
        void sendCompletedKnot(std_msgs::Header& header);
        bool pathToPod(platform_motion_msgs::Knot &previous, platform_motion_msgs::Knot &current, platform_motion_msgs::Knot &next, Eigen::Vector3d pod_pos, double &lastValidSteeringAngle, PodSegment *retval);
        void moreDataNeededCallback(CANOpen::DS301 &node);
        void errorCallback(CANOpen::DS301 &node);

        void sendPvtSegment(); // send next pvt segment to all wheelpods
        void setLastSegmentToCurrent( void );
        void primePVT(void);

        // debug print statement variable. probably shouldn't be committed
        int m_debugPrintCounter;

        ros::NodeHandle nh_;
        ros::NodeHandle param_nh;


        tf::TransformListener listener;
        std::string child_frame_id;

        ros::Subscriber planner_sub;
        ros::Subscriber joystick_sub;
        ros::Subscriber servo_sub;
        ros::Subscriber carousel_sub;
        ros::Subscriber gpio_sub;
        ros::Subscriber planned_path_sub;
        ros::Publisher gpio_pub;
        ros::Publisher joint_state_pub;
        ros::Publisher battery_voltage_pub;
        ros::Publisher status_pub;
        ros::Publisher stitchedPath_pub_;
        ros::Publisher motionMode_pub_;
        ros::Publisher completed_pub_;

        ros::Timer status_publish_timer;

        dynamic_reconfigure::Server<PlatformParametersConfig>
            reconfigure_server;

        actionlib::SimpleActionServer<platform_motion_msgs::HomeAction> home_pods_action_server;
        platform_motion_msgs::HomeFeedback home_pods_feedback;
        platform_motion_msgs::HomeResult home_pods_result;
        int home_pods_count;

        actionlib::SimpleActionServer<platform_motion_msgs::HomeAction> home_carousel_action_server;
        platform_motion_msgs::HomeFeedback home_carousel_feedback;
        platform_motion_msgs::HomeResult home_carousel_result;

        ros::ServiceServer enable_pods_server;
        int enable_pods_count;
        boost::mutex enable_pods_mutex;
        boost::condition_variable enable_pods_cond;

        ros::ServiceServer enable_carousel_server;
        boost::mutex enable_carousel_mutex;
        boost::condition_variable enable_carousel_cond;

        ros::ServiceServer motion_mode_server;
        int motion_mode, saved_mode;

        boost::mutex lock_state_mutex_;
        boost::condition_variable lock_state_cond_;

        std::tr1::shared_ptr<WheelPod> port, starboard, stern;
        std::tr1::shared_ptr<CANOpen::CopleyServo> carousel;

        Eigen::Vector3d port_pos_, starboard_pos_, stern_pos_;

        double wheel_diameter;

        double center_pt_x, center_pt_y;
        double min_wheel_speed;

        double planToZeroDecel_, planToZeroPeriod_;

        double portSteeringLock_, starboardSteeringLock_, sternSteeringLock_;
        double steeringTolerance_, steeringAccel_;

        double maxWheelAcceleration_, maxSteeringVelocity_;

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
        bool targetReached_;
        boost::mutex target_reached_mutex_;

        Eigen::Vector2d body_pt;
        int pv_counter;
        int joint_seq;

        // pvt mode members
        bool moreDataSent; // send more data once per sync.
        bool restartPvt; // need to send 3 segments
        bool newPathReady;
        std::list<platform_motion_msgs::Knot> plannedPath;
        BodySegment firstSegment_, secondSegment_, lastSegmentSent_; // the last segment we actually gave the wheelpods
        // keep track of the last valid steering angle computed by path to pod. this is important
        // to avoid flipping steering angle all the damn time!
        double lastValidSternSteeringAngle, lastValidPortSteeringAngle, lastValidStarboardSteeringAngle;
        std::mutex path_mutex_;
};

}


