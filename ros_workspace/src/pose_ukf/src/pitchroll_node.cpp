#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pose_ukf/PitchRoll.h>
#include <pose_ukf/pitchroll_ukf.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <functional>


namespace PitchRollUKF {

class PitchRollUKFNode
{
    std::string base_name_;
    double publish_period_;
    int seq_;

    void imuCallback(sensor_msgs::ImuConstPtr msg);
    void sendPose(const ros::TimerEvent& e);

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    ros::Subscriber imu_subscription_;

    ros::Timer publish_timer_;
    ros::Publisher pose_pub_;
    ros::Publisher state_pub_;

    PitchRollUKF *ukf_;
    ros::Time last_update_;
    ros::Time last_joint_state_;
    ros::Time last_imu_;
    std::string imu_frame_;
    bool first_update_;

    Eigen::Vector3d sigma_orientation_;
    Eigen::Vector3d sigma_omega_;
    Eigen::Vector2d sigma_gyro_bias_;
    Eigen::Vector3d sigma_accel_bias_;

    void parseProcessSigma(const ros::NodeHandle& privatenh);
    Eigen::MatrixXd process_noise(double dt) const;
    void sendState(void);

    public:
        PitchRollUKFNode();
        ~PitchRollUKFNode();
        void connect(void);
        void printState(void);
};

PitchRollUKFNode::PitchRollUKFNode() :
    ukf_(NULL),
    first_update_(true)
{
    ros::NodeHandle privatenh("~");
    ros::NodeHandle nh("~");

    privatenh.param("base_name", base_name_, std::string("base_link"));

    privatenh.param("publish_period", publish_period_, 0.020);

    double alpha, beta, kappa;
    privatenh.param("alpha", alpha, 1e-3);
    privatenh.param("beta", beta, 2.0);
    privatenh.param("kappa", kappa, 0.0);

    ukf_ = new PitchRollUKF(alpha, beta, kappa);
    parseProcessSigma(privatenh);

    std::vector<double> cov_vect;
    Eigen::MatrixXd initial_covariance;
    if(privatenh.getParam("initial_covariance", cov_vect))
    {
        int ndim = PitchRollState().ndim();
        initial_covariance = Eigen::MatrixXd::Map(cov_vect.data(), ndim, ndim);
        PitchRollState st = ukf_->state();
        ukf_->reset(st, initial_covariance);
    }

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
    state_pub_ = nh.advertise<pose_ukf::PitchRoll>("state", 1);
    seq_ = 0;
    publish_timer_ = privatenh.createTimer(ros::Duration(publish_period_), &PitchRollUKFNode::sendPose, this);
}

void
PitchRollUKFNode::connect(void)
{
    ros::NodeHandle nh;
    imu_subscription_ = nh.subscribe("imu", 1, &PitchRollUKFNode::imuCallback, this);
    ROS_INFO("Subscribers created");
    publish_timer_.start();
}

PitchRollUKFNode::~PitchRollUKFNode()
{
}

void
PitchRollUKFNode::sendPose(const ros::TimerEvent& e)
{
    (void)e;
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped());

    msg->header.frame_id = imu_frame_;
    msg->header.stamp = ros::Time::now();
    msg->header.seq = seq_++;
    tf::pointEigenToMsg( Eigen::Vector3d::Zero(), msg->pose.position);
    tf::quaternionEigenToMsg( ukf_->state().Orientation, msg->pose.orientation);
    pose_pub_.publish(msg);
}

void
PitchRollUKFNode::sendState(void)
{
    pose_ukf::PitchRollPtr state(new pose_ukf::PitchRoll());
    state->header.frame_id = imu_frame_;
    state->header.stamp = ros::Time::now();
    tf::quaternionEigenToMsg(ukf_->state().Orientation, state->orientation);
    state->omega.x = ukf_->state().Omega(0);
    state->omega.y = ukf_->state().Omega(1);
    state->omega.z = ukf_->state().Omega(2);
    //tf::vectorEigenToMsg(ukf_->state().AccelBias, state->accel_bias);
    state->gyro_bias.x = ukf_->state().GyroBias(0);
    state->gyro_bias.y = ukf_->state().GyroBias(1);
    state_pub_.publish(state);
}

void
PitchRollUKFNode::imuCallback(sensor_msgs::ImuConstPtr msg)
{
    if(first_update_)
    {
        first_update_ = false;
        last_update_ = ros::Time::now();
        last_imu_ = msg->header.stamp;
        imu_frame_ = msg->header.frame_id;
    }
    IMUOrientationMeasurement m;
    tf::vectorMsgToEigen(msg->linear_acceleration,
                         m.acceleration);
    m.acceleration *= m.littleg;
    Eigen::Vector3d tmpomega;
    tf::vectorMsgToEigen(msg->angular_velocity,
                         m.omega);
    m.delta_t = (msg->header.stamp - last_imu_).toSec();
    last_imu_ = msg->header.stamp;
    Eigen::MatrixXd meas_cov(m.ndim(), m.ndim());
    meas_cov.setZero();
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > accel_cov(msg->linear_acceleration_covariance.data());
    meas_cov.block<3, 3>(0, 0) = accel_cov;
    ROS_INFO_STREAM("imu accel cov:\n" << accel_cov);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > omega_cov(msg->angular_velocity_covariance.data());
    if(m.use_yaw_)
    {
        meas_cov.block<3, 3>(3, 3) = omega_cov;
    }
    else
    {
        meas_cov.block<2, 2>(3, 3) = omega_cov.block<2,2>(0,0);
    }
    ROS_INFO_STREAM("imu omega cov:\n" << omega_cov);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt<0)
        return;
    ROS_INFO_STREAM("Performing IMU predict with dt=" << dt << ", delta_t=" << m.delta_t);
    ROS_INFO_STREAM("process noise:\n" << process_noise(dt));
    ukf_->predict(dt, process_noise(dt));
    last_update_ = msg->header.stamp;
    ROS_INFO_STREAM("Performing IMU correct with measurement:\n" << m);
    ROS_INFO_STREAM("Measurement covariance :\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    printState();
    sendState();
}

void
PitchRollUKFNode::printState(void)
{
    ROS_INFO_STREAM("State:\n" << ukf_->state());
    ROS_INFO_STREAM("Covariance:\n" << ukf_->covariance());
    if(ukf_->state().use_yaw_)
    {
        ROS_INFO_STREAM("orientation cov:\n" << (ukf_->covariance().block<3,3>(0,0)));
        ROS_INFO_STREAM("omega cov:\n" << (ukf_->covariance().block<3,3>(3,3)));
        ROS_INFO_STREAM("gyro bias cov:\n" << (ukf_->covariance().block<2,2>(6,6)));
    }
    else
    {
        ROS_INFO_STREAM("orientation cov:\n" << (ukf_->covariance().block<2,2>(0,0)));
        ROS_INFO_STREAM("omega cov:\n" << (ukf_->covariance().block<2,2>(2,2)));
        ROS_INFO_STREAM("gyro bias cov:\n" << (ukf_->covariance().block<2,2>(4,4)));
    }
    //ROS_INFO_STREAM("accel bias cov:\n" << (ukf_->covariance().block<3,3>(6,6)));
}

template <typename V>
void listToVec(XmlRpc::XmlRpcValue& list, V *vec)
{
    int s = vec->size();
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i<std::min(s, list.size()); i++)
    {
        if(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
           list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            (*vec)(i) = static_cast<double>(list[i]);
        }
        else if(list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            (*vec)(i) = atof(static_cast<std::string>(list[i]).c_str());
        }
    }
}

Eigen::MatrixXd
PitchRollUKFNode::process_noise(double dt) const
{
    Eigen::MatrixXd noise(ukf_->ndim(), ukf_->ndim());
    noise.setZero();

    // position, velocity, orientation, omega, gyro_bias, accel_bias
    if(ukf_->state().use_yaw_)
    {
        noise.block<3,3>(0,0).diagonal() = (dt*sigma_orientation_).cwiseProduct(dt*sigma_orientation_);
        noise.block<3,3>(0,3).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
        noise.block<3,3>(3,0).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
        noise.block<3,3>(3,3).diagonal() = (dt*sigma_omega_).cwiseProduct(dt*sigma_omega_);
        noise.block<2,2>(6,6).diagonal() = (dt*sigma_gyro_bias_).cwiseProduct(dt*sigma_gyro_bias_);
    }
    else
    {
        noise.block<2,2>(0,0).diagonal() = (dt*sigma_orientation_.segment<2>(0)).cwiseProduct(dt*sigma_orientation_.segment<2>(0));
        noise.block<2,2>(0,2).diagonal() = (dt*dt*sigma_omega_.segment<2>(0)/2.).cwiseProduct(dt*dt*sigma_omega_.segment<2>(0)/2.);
        noise.block<2,2>(2,0).diagonal() = (dt*dt*sigma_omega_.segment<2>(0)/2.).cwiseProduct(dt*dt*sigma_omega_.segment<2>(0)/2.);
        noise.block<2,2>(2,2).diagonal() = (dt*sigma_omega_.segment<2>(0)).cwiseProduct(dt*sigma_omega_.segment<2>(0));
        noise.block<2,2>(4,4).diagonal() = (dt*sigma_gyro_bias_).cwiseProduct(dt*sigma_gyro_bias_);
    }
    //noise.block<3,3>(6,6).diagonal() = (dt*sigma_accel_bias_).cwiseProduct(dt*sigma_accel_bias_);

    return noise;
}

void
PitchRollUKFNode::parseProcessSigma(const ros::NodeHandle& privatenh)
{
    if(!privatenh.hasParam("process_sigma"))
    {
        ROS_ERROR("No process noise sigma specified");
        exit(1);
    }
    XmlRpc::XmlRpcValue process_sigma;
    privatenh.getParam("process_sigma", process_sigma);
    ROS_ASSERT(process_sigma.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(process_sigma.hasMember(std::string("orientation")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("orientation")];
        listToVec(list, &sigma_orientation_);
    }
    if(process_sigma.hasMember(std::string("omega")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("omega")];
        listToVec(list, &sigma_omega_);
    }
    if(process_sigma.hasMember(std::string("gyro_bias")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("gyro_bias")];
        listToVec(list, &sigma_gyro_bias_);
    }
    if(process_sigma.hasMember(std::string("accel_bias")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("accel_bias")];
        listToVec(list, &sigma_accel_bias_);
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PitchRollUKF");
    PitchRollUKF::PitchRollUKFNode n;
    n.connect();
    ros::spin();
}