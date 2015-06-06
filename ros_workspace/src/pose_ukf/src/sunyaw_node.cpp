#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "pose_ukf/PitchRoll.h"
#include "pose_ukf/sunyaw_ukf.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <functional>
#include <visualization_msgs/Marker.h>

#include "solar_fisheye/SunSensor.h"



namespace SunYawUKF {

class SunYawUKFNode
{
    std::string base_name_;
    double publish_period_;
    int seq_;

    void sunCallback(const solar_fisheye::SunSensorConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void sendPose(const ros::TimerEvent& e);

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    ros::Subscriber imu_subscription_;
    ros::Subscriber sun_sub_;

    ros::Timer publish_timer_;
    ros::Publisher pose_pub_;
    ros::Publisher state_pub_;
    ros::Publisher vis_pub_;

    SunYawUKF *ukf_;
    ros::Time last_update_;
    ros::Time last_joint_state_;
    ros::Time last_imu_;
    std::string imu_frame_;
    bool first_update_;

    Eigen::Vector3d sigma_orientation_;
    Eigen::Vector3d sigma_omega_;
    Eigen::Vector2d sigma_gyro_bias_;
    Eigen::Vector3d sigma_accel_bias_;
    double sigma_sun_;

    void parseProcessSigma(const ros::NodeHandle& privatenh);
    Eigen::MatrixXd process_noise(double dt) const;
    void sendState(void);

    public:
        SunYawUKFNode();
        ~SunYawUKFNode();
        void connect(void);
        void printState(std::string name="stateprint");
};

SunYawUKFNode::SunYawUKFNode() :
    ukf_(NULL),
    first_update_(true)
{
    ros::NodeHandle privatenh("~");
    ros::NodeHandle nh("~");

    privatenh.param("base_name", base_name_, std::string("base_link"));

    privatenh.param("publish_period", publish_period_, 0.020);

    privatenh.param("sigma_sun", sigma_sun_, 0.05);

    double alpha, beta, kappa;
    privatenh.param("alpha", alpha, 1e-3);
    privatenh.param("beta", beta, 2.0);
    privatenh.param("kappa", kappa, 0.0);

    ukf_ = new SunYawUKF(alpha, beta, kappa);
    parseProcessSigma(privatenh);

    std::vector<double> cov_vect;
    Eigen::MatrixXd initial_covariance;
    if(privatenh.getParam("initial_covariance", cov_vect))
    {
        int ndim = SunYawState().ndim();
        initial_covariance = Eigen::MatrixXd::Map(cov_vect.data(), ndim, ndim);
        SunYawState st = ukf_->state();
        ukf_->reset(st, initial_covariance);
    }

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
    state_pub_ = nh.advertise<pose_ukf::PitchRoll>("state", 1);
    vis_pub_ = nh.advertise<visualization_msgs::Marker>("sun_vis", 1);
    seq_ = 0;
    publish_timer_ = privatenh.createTimer(ros::Duration(publish_period_), &SunYawUKFNode::sendPose, this);
}

void
SunYawUKFNode::connect(void)
{
    ros::NodeHandle nh;
    imu_subscription_ = nh.subscribe("imu", 1, &SunYawUKFNode::imuCallback, this);
    sun_sub_ = nh.subscribe("sun", 1, &SunYawUKFNode::sunCallback, this);
    ROS_INFO("Subscribers created");
    publish_timer_.start();
}

SunYawUKFNode::~SunYawUKFNode()
{
}

void
SunYawUKFNode::sendPose(const ros::TimerEvent& e)
{
    (void)e;
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped());

    ros::Time now = ros::Time::now();
    if(listener_.canTransform("map", imu_frame_, ros::Time(0)))
    {
        msg->header.frame_id = "map";
        msg->header.stamp = ros::Time::now();
        msg->header.seq = seq_++;
        tf::StampedTransform imuInMap;
        listener_.lookupTransform("map", imu_frame_, ros::Time(0), imuInMap);
        tf::StampedTransform baseInImu;
        listener_.lookupTransform(imu_frame_, base_name_, ros::Time(0), baseInImu);
        Eigen::Quaterniond brot;
        tf::quaternionTFToEigen(baseInImu.getRotation(), brot);
        tf::pointTFToMsg( imuInMap.getOrigin(), msg->pose.position);
        tf::quaternionEigenToMsg( brot*ukf_->state().Orientation.inverse(), msg->pose.orientation);
        pose_pub_.publish(msg);
    }
}

void
SunYawUKFNode::sendState(void)
{
    pose_ukf::PitchRollPtr state(new pose_ukf::PitchRoll());
    state->header.frame_id = imu_frame_;
    state->header.stamp = ros::Time::now();
    tf::quaternionEigenToMsg(ukf_->state().Orientation, state->orientation);
    state->omega.x = ukf_->state().Omega(0);
    state->omega.y = ukf_->state().Omega(1);
    state->omega.z = ukf_->state().Omega(2);
    state->gyro_bias.x = ukf_->state().GyroBias(0);
    state->gyro_bias.y = ukf_->state().GyroBias(1);
    state->gyro_bias.z = ukf_->state().GyroBias(2);
    state_pub_.publish(state);
}

void
SunYawUKFNode::imuCallback(const sensor_msgs::ImuConstPtr& msg)
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
    ROS_DEBUG_STREAM_NAMED("imucallback", "imu accel cov:\n" << accel_cov);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > omega_cov(msg->angular_velocity_covariance.data());
    meas_cov.block<3, 3>(3, 3) = omega_cov;
    ROS_DEBUG_STREAM_NAMED("imucallback", "imu omega cov:\n" << omega_cov);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt<0)
        return;
    ROS_DEBUG_STREAM_NAMED("imucallback", "Performing IMU predict with dt=" << dt << ", delta_t=" << m.delta_t);
    ROS_DEBUG_STREAM_NAMED("imucallback", "process noise:\n" << process_noise(dt));
    ukf_->predict(dt, process_noise(dt));
    last_update_ = msg->header.stamp;
    ROS_DEBUG_STREAM_NAMED("imucallback", "Performing IMU correct with measurement:\n" << m);
    ROS_DEBUG_STREAM_NAMED("imucallback", "Measurement covariance :\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    printState("imucallback");
    sendState();
}

void
SunYawUKFNode::sunCallback(const solar_fisheye::SunSensorConstPtr& msg)
{
    SunSensorMeasurement m;
    tf::vectorMsgToEigen(msg->reference,
                         m.reference);
    geometry_msgs::Vector3Stamped vcamera, vimu;
    vcamera.header = msg->header;
    vcamera.vector = msg->measurement;
    std::string errmsg;
    if(!listener_.canTransform(imu_frame_, msg->header.frame_id, msg->header.stamp, &errmsg))
    {
        ROS_ERROR_STREAM_NAMED("suncallback", "Cannot transform " << msg->header.frame_id << " to " << imu_frame_ << ": " << errmsg);
        return;
    }
    tf::StampedTransform baseInImu;
    if(!listener_.canTransform(imu_frame_, base_name_, ros::Time(0), &errmsg))
    {
        ROS_ERROR_STREAM_NAMED("suncallback", "Cannot transform " << base_name_ << " to " << imu_frame_ << ": " << errmsg);
        return;
    }

    listener_.lookupTransform(imu_frame_, base_name_, ros::Time(0), baseInImu);
    Eigen::Quaterniond brot;
    tf::quaternionTFToEigen(baseInImu.getRotation(), brot);

    listener_.transformVector(imu_frame_, vcamera, vimu);
    tf::vectorMsgToEigen(vimu.vector,
                         m.measurement);
    m.measurement = brot*ukf_->state().Orientation.inverse()*m.reference;
    Eigen::MatrixXd meas_cov(m.ndim(), m.ndim());
    meas_cov.setZero();
    meas_cov(0,0) = sigma_sun_;
    meas_cov(1,1) = sigma_sun_;
    meas_cov(2,2) = sigma_sun_;
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    if(listener_.canTransform("map", imu_frame_, ros::Time(0), &errmsg))
    {
        tf::StampedTransform imuInMap;
        listener_.lookupTransform("map", imu_frame_, ros::Time(0), imuInMap);
        visualization_msgs::Marker meas_mkr;
        meas_mkr.header.stamp = msg->header.stamp;
        meas_mkr.header.frame_id = "map";
        meas_mkr.id = 0;
        meas_mkr.type = visualization_msgs::Marker::ARROW;
        meas_mkr.action = visualization_msgs::Marker::MODIFY;
        tf::pointTFToMsg(imuInMap.getOrigin(), meas_mkr.pose.position);
        Eigen::Quaterniond qor = Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1,0,0),
                ukf_->state().Orientation*m.measurement);
        tf::quaternionEigenToMsg(qor, meas_mkr.pose.orientation);
        meas_mkr.scale.x=3.0;
        meas_mkr.scale.y=0.1;
        meas_mkr.scale.z=0.1;
        meas_mkr.color.r=1.0;
        meas_mkr.color.g=1.0;
        meas_mkr.color.b=0.0;
        meas_mkr.color.a=1.0;
        meas_mkr.text="measurement";
        vis_pub_.publish(meas_mkr);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("suncallback", "Cannot transform " << imu_frame_ << " to " << "map" << ": " << errmsg);
    }

    ROS_DEBUG_STREAM_NAMED("suncallback", "Performing SUN correct with measurement:\n" << m);
    ROS_DEBUG_STREAM_NAMED("suncallback", "Measurement covariance :\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    printState("suncallback");
    sendState();
}

void
SunYawUKFNode::printState(std::string name)
{
    ROS_DEBUG_STREAM_NAMED(name, "State:\n" << ukf_->state());
    ROS_DEBUG_STREAM_NAMED(name, "Covariance:\n" << ukf_->covariance());
    ROS_DEBUG_STREAM_NAMED(name, "orientation cov:\n" << (ukf_->covariance().block<3,3>(0,0)));
    ROS_DEBUG_STREAM_NAMED(name, "omega cov:\n" << (ukf_->covariance().block<3,3>(3,3)));
    ROS_DEBUG_STREAM_NAMED(name, "gyro bias cov:\n" << (ukf_->covariance().block<3,3>(6,6)));
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
SunYawUKFNode::process_noise(double dt) const
{
    Eigen::MatrixXd noise(ukf_->ndim(), ukf_->ndim());
    noise.setZero();

    // position, velocity, orientation, omega, gyro_bias, accel_bias
    noise.block<3,3>(0,0).diagonal() = (dt*sigma_orientation_).cwiseProduct(dt*sigma_orientation_);
    noise.block<3,3>(0,3).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
    noise.block<3,3>(3,0).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
    noise.block<3,3>(3,3).diagonal() = (dt*sigma_omega_).cwiseProduct(dt*sigma_omega_);
    noise.block<2,2>(6,6).diagonal() = (dt*sigma_gyro_bias_).cwiseProduct(dt*sigma_gyro_bias_);

    return noise;
}

void
SunYawUKFNode::parseProcessSigma(const ros::NodeHandle& privatenh)
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
    ros::init(argc, argv, "SunYawUKF");
    SunYawUKF::SunYawUKFNode n;
    n.connect();
    ros::spin();
}
