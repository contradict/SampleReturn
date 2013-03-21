#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

namespace platform_motion
{

class OdometryNode {
    public:
        OdometryNode(void);
        void init(void);

    private:
        bool lookupJointValue(const sensor_msgs::JointState::ConstPtr &joint_state,
                std::string requested_name,
                double *position, double *velocity);
        void fillOdoMsg(nav_msgs::Odometry *odo, ros::Time stamp, bool stopped);
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr joint_state);

        ros::NodeHandle nh_;
        tf::TransformListener listener;

        ros::Subscriber joint_state_sub;

        tf::TransformBroadcaster odom_broadcaster;
        ros::Publisher odometry_pub;

        bool first;
        ros::Time last_joint_message;
        double last_starboard_wheel, last_port_wheel, last_stern_wheel;
        double starboard_vel_sum, port_vel_sum, stern_vel_sum;
        int vel_sum_count;
        Eigen::Vector2d odom_position;
        double odom_orientation;
        Eigen::Vector2d odom_velocity;
        double odom_omega;
        double wheel_diameter;
        double delta_threshold;
        double min_translation_norm;
        double unexplainable_jump;
        std::string odom_frame_id, child_frame_id;
};

}
