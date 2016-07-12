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
    void gyroCallback(sensor_msgs::ImuConstPtr msg);
    void sendPose(void);

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    ros::Subscriber imu_subscription_;
    ros::Subscriber gyro_subscription_;

    //ros::Timer publish_timer_;
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
    Eigen::Vector3d sigma_gyro_bias_;
    Eigen::Vector3d sigma_accel_bias_;

    void parseProcessSigma(const ros::NodeHandle& privatenh);
    Eigen::MatrixXd process_noise(double dt) const;
    void sendState(void);
    bool lookupTransform(std::string frame_id, std::string to_id, tf::StampedTransform& transform);

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

    privatenh.param("base_name", base_name_, std::string("base_link"));

    privatenh.param("publish_period", publish_period_, 0.020);

    double alpha, beta, kappa;
    privatenh.param("alpha", alpha, 1e-3);
    privatenh.param("beta", beta, 2.0);
    privatenh.param("kappa", kappa, 0.0);

    ukf_ = new PitchRollUKF(alpha, beta, kappa);
    parseProcessSigma(privatenh);

    std::vector<double> cov_vect;
    if(privatenh.getParam("initial_covariance", cov_vect))
    {
        int ndim = PitchRollState().ndim();
        PitchRollState st = ukf_->state();
        st.Covariance = Eigen::MatrixXd::Map(cov_vect.data(), ndim, ndim);
        ukf_->reset(st);
    }
    else
    {
        ROS_WARN_STREAM("No initial covariance specified, using identity");
    }

    pose_pub_ =privatenh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
    state_pub_ = privatenh.advertise<pose_ukf::PitchRoll>("state", 1);
    seq_ = 0;
    ROS_INFO_STREAM("Initial state:\n" << ukf_->state());
    ROS_INFO_STREAM("Initial Cov:\n" << ukf_->state().Covariance);
}

void
PitchRollUKFNode::connect(void)
{
    ros::NodeHandle nh;

    ROS_DEBUG_STREAM("Waiting for gyro transform");

    std::string err="Waiting for gyro->imu";
    while(!listener_.waitForTransform("gyro", "imu_0", ros::Time(0), ros::Duration(1), ros::Duration(0.01), &err)
            && ros::ok());

    imu_subscription_ = nh.subscribe("imu", 1, &PitchRollUKFNode::imuCallback, this);
    gyro_subscription_ = nh.subscribe("gyro", 1, &PitchRollUKFNode::gyroCallback, this);
    ROS_INFO("Subscribers created");
    //publish_timer_.start();

}

PitchRollUKFNode::~PitchRollUKFNode()
{
}

void
PitchRollUKFNode::sendPose(void)
{
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped());

    msg->header.frame_id = imu_frame_;
    msg->header.stamp = ros::Time::now();
    msg->header.seq = seq_++;
    tf::pointEigenToMsg( Eigen::Vector3d::Zero(), msg->pose.position);
    tf::quaternionEigenToMsg( ukf_->state().Orientation.unit_quaternion(), msg->pose.orientation);
    pose_pub_.publish(msg);
}

void
PitchRollUKFNode::sendState(void)
{
    pose_ukf::PitchRollPtr state(new pose_ukf::PitchRoll());
    state->header.frame_id = imu_frame_;
    state->header.stamp = ros::Time::now();
    tf::quaternionEigenToMsg(ukf_->state().Orientation.unit_quaternion(), state->orientation);
    state->omega.x = ukf_->state().Omega(0);
    state->omega.y = ukf_->state().Omega(1);
    state->omega.z = ukf_->state().Omega(2);
    tf::vectorEigenToMsg(ukf_->state().AccelBias, state->accel_bias);
    state->gyro_bias.x = ukf_->state().GyroBias(0);
    state->gyro_bias.y = ukf_->state().GyroBias(1);
    state->gyro_bias.x = ukf_->state().GyroBias(2);
    state_pub_.publish(state);
}

bool PitchRollUKFNode::lookupTransform(std::string frame_id, std::string to_id, tf::StampedTransform& transform)
{
    if(frame_id=="" || to_id=="")
    {
        ROS_WARN_STREAM("Missing frame id \"" << frame_id << "\" -> \"" << to_id << "\"");
        return false;
    }
    try
    {
        listener_.lookupTransform(frame_id,
                to_id,
                ros::Time(0),
                transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR_STREAM("Unable to look up " << frame_id << "->" << to_id << ": " << ex.what());
        return false;
    }
    return true;
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
        ROS_DEBUG_STREAM("Set imu_frame_ " << imu_frame_);
    }
    IMUOrientationMeasurement m;
    tf::vectorMsgToEigen(msg->linear_acceleration,
                         m.acceleration);
    m.acceleration.normalize();
    Eigen::Vector3d tmpomega;
    tf::vectorMsgToEigen(msg->angular_velocity,
                         m.omega);
    last_imu_ = msg->header.stamp;
    Eigen::MatrixXd meas_cov(m.ndim(), m.ndim());
    meas_cov.setZero();
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > accel_cov(msg->linear_acceleration_covariance.data());
    meas_cov.block<3, 3>(0, 0) = accel_cov;
    ROS_DEBUG_STREAM("imu accel cov:\n" << accel_cov);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > omega_cov(msg->angular_velocity_covariance.data());
    meas_cov.block<3, 3>(3, 3) = omega_cov;
    ROS_DEBUG_STREAM("imu omega cov:\n" << omega_cov);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>0)
    {
        ROS_DEBUG_STREAM("Performing imu predict with dt=" << dt );
        ROS_DEBUG_STREAM("process noise:\n" << process_noise(dt));
        ukf_->predict(dt, process_noise(dt));
        last_update_ = msg->header.stamp;
    }
    else {
        ROS_DEBUG_STREAM("Skipping imu predict dt=" << dt);
    }
    ROS_DEBUG_STREAM("Performing IMU correct with measurement:\n" << m);
    ROS_DEBUG_STREAM("Measurement covariance :\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    printState();
    sendState();
}

void
PitchRollUKFNode::gyroCallback(sensor_msgs::ImuConstPtr msg)
{
    ROS_DEBUG_STREAM("Got gyro msg " << msg->header.seq);
    tf::StampedTransform gyro_transform;
    if(!lookupTransform(msg->header.frame_id, imu_frame_, gyro_transform))
    {
       return;
    }

    tf::Quaternion gq;
    tf::quaternionMsgToTF(msg->orientation, gq);
    tf::Transform gyro(gq);
    tf::Transform gyro_imu = gyro_transform*gyro*gyro_transform.inverse();
    Eigen::Quaterniond imu_gq;
    tf::quaternionTFToEigen(gyro.getRotation(), imu_gq);
    YawMeasurement m;
    m.yaw = Sophus::SO3d(imu_gq);
    Eigen::MatrixXd meas_cov(m.ndim(), m.ndim());
    meas_cov.setZero();
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov(msg->orientation_covariance.data());
    meas_cov(0,0) = cov(2,2);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>0)
    {
        ROS_DEBUG_STREAM("Performing gyro predict with dt=" << dt );
        ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
        ukf_->predict(dt, process_noise(dt));
        ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
        last_update_ = msg->header.stamp;
    }
    else
    {
        ROS_DEBUG_STREAM("Skipping gyro predict dt=" << dt);
    }

    ROS_DEBUG_STREAM("GYRO correct:\n" << m);
    ROS_DEBUG_STREAM("GYRO measurement covariance:\n" << meas_cov);
    ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
    ukf_->correct(m, meas_covs);
    ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
    sendState();
}

void
PitchRollUKFNode::printState(void)
{
    PitchRollState st = ukf_->state();
    ROS_INFO_STREAM("State:\n" << st);
    ROS_INFO_STREAM("Covariance:\n" << st.Covariance);
    ROS_INFO_STREAM("orientation cov:\n" << (st.Covariance.block<3,3>(0,0)));
    ROS_INFO_STREAM("omega cov:\n" << (st.Covariance.block<3,3>(3,3)));
    ROS_INFO_STREAM("gyro bias cov:\n" << (st.Covariance.block<3,3>(6,6)));
    ROS_INFO_STREAM("accel bias cov:\n" << (st.Covariance.block<3,3>(9,9)));
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
    noise.block<3,3>(0,0).diagonal() = (dt*sigma_orientation_).cwiseProduct(dt*sigma_orientation_);
    noise.block<3,3>(0,3).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
    noise.block<3,3>(3,0).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
    noise.block<3,3>(3,3).diagonal() = (dt*sigma_omega_).cwiseProduct(dt*sigma_omega_);
    noise.block<3,3>(6,6).diagonal() = (dt*sigma_gyro_bias_).cwiseProduct(dt*sigma_gyro_bias_);
    noise.block<3,3>(9,9).diagonal() = (dt*sigma_accel_bias_).cwiseProduct(dt*sigma_accel_bias_);

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