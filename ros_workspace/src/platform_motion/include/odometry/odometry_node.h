#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

namespace platform_motion
{

struct odometry_measurements {
    Eigen::Vector2d body_pt;
    Eigen::Vector2d port_pos, port_dir;
    double port_distance;
    double port_delta;
    double port_vel;
    double port_angle;
    Eigen::Vector2d stern_pos, stern_dir;
    double stern_distance;
    double stern_delta;
    double stern_vel;
    double stern_angle;
    Eigen::Vector2d starboard_pos, starboard_dir;
    double starboard_distance;
    double starboard_delta;
    double starboard_vel;
    double starboard_angle;
    double interval;
    double min_translation_norm;
};
std::ostream & operator<< (std::ostream &out, const odometry_measurements &m);

class OdometryNode {
    public:
        OdometryNode(void);
        virtual void init(void);

    protected:
        bool lookupJointValue(const sensor_msgs::JointState::ConstPtr &joint_state,
                std::string requested_name,
                double *position, double *velocity);
        void fillDefaultCovariance(void);
        void fillOdoMsg(nav_msgs::Odometry *odo, ros::Time stamp, bool stopped);
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr joint_state);
        bool isMoving(const struct odometry_measurements &data);
        void getPodPositions(struct odometry_measurements *data);
        void fillMeasurements(const sensor_msgs::JointState::ConstPtr joint_state, struct odometry_measurements *data);
        bool detectJump(const struct odometry_measurements &data, ros::Time stamp);
        void resetReferencePose(const struct odometry_measurements &data, const ros::Time &stamp);
        virtual void computeOdometry(struct odometry_measurements &data, const ros::Time &stamp)=0;

        ros::NodeHandle nh_;
        tf::TransformListener listener;

        ros::Subscriber joint_state_sub;

        bool publish_tf;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Publisher odometry_pub;

        bool first;
        ros::Time last_joint_message;
        double last_starboard_distance, last_port_distance, last_stern_distance;
        double starboard_vel_sum, port_vel_sum, stern_vel_sum;
        int vel_sum_count;

        Eigen::Vector2d odom_position;
        double odom_orientation;
        Eigen::Vector2d odom_velocity;
        double odom_omega;
        Eigen::VectorXd odom_pose_covariance;
        Eigen::VectorXd odom_twist_covariance;
        Eigen::VectorXd odom_twist_stopped_covariance;

        double wheel_diameter;
        double delta_threshold;
        double min_translation_norm;
        double unexplainable_jump;
        std::string odom_frame_id, child_frame_id;

        struct odometry_measurements data;

};

}
