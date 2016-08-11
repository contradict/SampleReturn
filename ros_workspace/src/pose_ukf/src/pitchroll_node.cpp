#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pose_ukf/PitchRoll.h>
#include <pose_ukf/pitchroll_ukf.hpp>
#include <pose_ukf/yaw_measurement.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <functional>


namespace PitchRollUKF {

class PitchRollUKFNode
{
    std::string odometry_frame_id_;
    std::string parent_frame_id_;
    std::string child_frame_id_;
    bool publish_tf_;
    int seq_;

    void imuCallback(sensor_msgs::ImuConstPtr msg);
    void gyroCallback(sensor_msgs::ImuConstPtr msg);
    void sendPose(void);
    void publishTFIdentity(void);

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
    double min_dt_;

    Eigen::Vector3d sigma_orientation_;
    Eigen::Vector3d sigma_omega_;
    Eigen::Vector3d sigma_gyro_bias_;
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

    privatenh.param("odometry_frame", odometry_frame_id_, std::string("odom"));
    privatenh.param("parent_frame", parent_frame_id_, std::string("base_link_flat"));
    privatenh.param("child_frame", child_frame_id_, std::string("base_link"));
    privatenh.param("publish_tf", publish_tf_, true);

    double alpha, beta, kappa;
    privatenh.param("alpha", alpha, 1e-3);
    privatenh.param("beta", beta, 2.0);
    privatenh.param("kappa", kappa, 0.0);
    privatenh.param("min_dt", min_dt_, 0.001);

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

    imu_subscription_ = nh.subscribe("imu", 1, &PitchRollUKFNode::imuCallback, this);
    gyro_subscription_ = nh.subscribe("gyro", 1, &PitchRollUKFNode::gyroCallback, this);
    ROS_INFO("Subscribers created");
    //publish_timer_.start();

}

PitchRollUKFNode::~PitchRollUKFNode()
{
}

void
PitchRollUKFNode::publishTFIdentity(void)
{
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = parent_frame_id_;
    odom_trans.child_frame_id = child_frame_id_;
    tf::Transform id;
    id.setIdentity();
    tf::transformTFToMsg(id, odom_trans.transform);
    broadcaster_.sendTransform(odom_trans);
}

void
PitchRollUKFNode::sendPose(void)
{
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped());

    msg->header.frame_id = imu_frame_;
    msg->header.stamp = ros::Time::now();
    msg->header.seq = seq_++;
    tf::pointEigenToMsg( Eigen::Vector3d::Zero(), msg->pose.position);
    tf::quaternionEigenToMsg( ukf_->state().Orientation.unit_quaternion().inverse(), msg->pose.orientation);
    pose_pub_.publish(msg);

    if(publish_tf_)
    {
        tf::StampedTransform odometry_in;
        tf::StampedTransform imu_trans;
        try
        {
            listener_.lookupTransform(parent_frame_id_, odometry_frame_id_, ros::Time(0), odometry_in);
            listener_.lookupTransform(child_frame_id_, imu_frame_, ros::Time(0), imu_trans);
        }
        catch(tf::TransformException ex)
        {
            //ROS_ERROR_STREAM("Unable to look up " << odometry_frame_id_ << "->" << parent_frame_id_ << ": " << ex.what());
            publishTFIdentity();
            return;
        }
        tf::Transform odometry_yaw;
        odometry_yaw.setIdentity();
        odometry_yaw.setRotation(odometry_in.getRotation());
        tf::Transform pose;
        tf::Quaternion pose_q;
        tf::quaternionEigenToTF(ukf_->state().Orientation.unit_quaternion(), pose_q);
        pose.setIdentity();
        pose.setRotation(pose_q);

        imu_trans.setOrigin(tf::Vector3(0,0,0));
        tf::Transform odometry_out = imu_trans*pose*imu_trans.inverse()*odometry_yaw;

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header = msg->header;
        odom_trans.header.frame_id = parent_frame_id_;
        odom_trans.child_frame_id = child_frame_id_;
        tf::transformTFToMsg(odometry_out, odom_trans.transform);
        broadcaster_.sendTransform(odom_trans);
    }
}

void
PitchRollUKFNode::sendState(void)
{
    pose_ukf::PitchRollPtr state(new pose_ukf::PitchRoll());
    state->header.frame_id = imu_frame_;
    state->header.stamp = ros::Time::now();
    tf::quaternionEigenToMsg(ukf_->state().Orientation.unit_quaternion(), state->orientation);
    tf::vectorEigenToMsg(ukf_->state().Omega, state->omega);
    tf::vectorEigenToMsg(ukf_->state().AccelBias, state->accel_bias);
    tf::vectorEigenToMsg(ukf_->state().GyroBias, state->gyro_bias);
    state_pub_.publish(state);
    //printState();
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
    //ROS_DEBUG_STREAM("imu accel cov:\n" << accel_cov);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > omega_cov(msg->angular_velocity_covariance.data());
    meas_cov.block<3, 3>(3, 3) = omega_cov;
    //ROS_DEBUG_STREAM("imu omega cov:\n" << omega_cov);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>min_dt_)
    {
        //ROS_DEBUG_STREAM("Performing imu predict with dt=" << dt );
        //ROS_DEBUG_STREAM("process noise:\n" << process_noise(dt));
        ukf_->predict(dt, process_noise(dt));
        last_update_ = msg->header.stamp;
    }
    else {
        ROS_DEBUG_STREAM("Skipping imu predict dt=" << dt);
    }
    //ROS_DEBUG_STREAM("Performing IMU correct with measurement:\n" << m);
    //ROS_DEBUG_STREAM("Measurement covariance :\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    sendPose();
    sendState();
}

void
PitchRollUKFNode::gyroCallback(sensor_msgs::ImuConstPtr msg)
{
    if(!listener_.canTransform(imu_frame_, msg->header.frame_id, msg->header.stamp))
    {
       if(!first_update_) publishTFIdentity();
       return;
    }

    tf::Quaternion gyro_q;
    tf::quaternionMsgToTF(msg->orientation, gyro_q);
    //ROS_INFO_STREAM("gyro_q: " <<
    //        gyro_q.x() << ", " <<
    //        gyro_q.y() << ", " <<
    //        gyro_q.z() << ", " <<
    //        gyro_q.w() << ", ");
    tf::StampedTransform gyro_imu;
    listener_.lookupTransform(imu_frame_, msg->header.frame_id, msg->header.stamp, gyro_imu);
    //ROS_INFO_STREAM("gyro_imu: " <<
    //        gyro_imu.getRotation().x() << ", " <<
    //        gyro_imu.getRotation().y() << ", " <<
    //        gyro_imu.getRotation().z() << ", " <<
    //        gyro_imu.getRotation().w() << ", ");
    tf::Quaternion gyro_imu_tf = gyro_imu.getRotation() * gyro_q * gyro_imu.getRotation().inverse();
    Eigen::Quaterniond imu_q;
    tf::quaternionTFToEigen(gyro_imu_tf, imu_q);
    //ROS_INFO_STREAM("imu_q: " <<
    //        imu_q.x() << ", " <<
    //        imu_q.y() << ", " <<
    //        imu_q.z() << ", " <<
    //        imu_q.w() << ", ");
    PoseUKF::YawMeasurement<PitchRollState> m;
    m.yaw = Sophus::SO3d(imu_q);
    Eigen::MatrixXd meas_cov(m.ndim(), m.ndim());
    meas_cov.setZero();
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov(msg->orientation_covariance.data());
    meas_cov(0,0) = cov(2,2);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>min_dt_)
    {
        //ROS_DEBUG_STREAM("Performing gyro predict with dt=" << dt );
        ukf_->predict(dt, process_noise(dt));
        last_update_ = msg->header.stamp;
    }
    else
    {
        ROS_DEBUG_STREAM("Skipping gyro predict dt=" << dt);
    }

    //ROS_DEBUG_STREAM("GYRO correct:\n" << m);
    //ROS_DEBUG_STREAM("GYRO measurement covariance:\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    sendState();
}

void
PitchRollUKFNode::printState(void)
{
    PitchRollState st = ukf_->state();
    ROS_INFO_STREAM("State:\n" << st);
    //ROS_INFO_STREAM("Covariance:\n" << st.Covariance);
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
