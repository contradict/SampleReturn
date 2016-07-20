#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pose_ukf/pose_ukf.hpp>
#include <pose_ukf/Pose.h>
#include <pose_ukf/yaw_measurement.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <functional>
#include <sophus/se3.hpp>


namespace PoseUKF {

bool lookupTransform(const tf::TransformListener& listener,
        std::string base_name,
        std::string link_name,
        std::string err,
        tf::StampedTransform* transform)
{
    std::string errmsg("Unknown error");
    if(!listener.waitForTransform(base_name, link_name, ros::Time(0),
                ros::Duration(5.0), ros::Duration(0.1), &errmsg))
    {
        ROS_ERROR("%s %s: %s", err.c_str(), link_name.c_str(), errmsg.c_str());
        return false;
    }
    listener.lookupTransform(base_name, link_name, ros::Time(0), *transform);
    return true;
}

class PoseUKFNode
{
    struct Wheel
    {
        Wheel(double d, double f, double v,
              double p,
              double large_rotation_jump, double large_steering_jump,
              std::string base_name=std::string("base_link")) :
            position_initialized(false),
            angle_initialized(false),
            large_rotation_jump(large_rotation_jump),
            large_steering_jump(large_steering_jump),
            diameter(d),
            forward_noise(f), perpendicular_noise(p),
            velocity_noise(v),
            base_name(base_name)
        {};
        tf::TransformListener listener;
        bool position_initialized, angle_initialized;
        double large_rotation_jump, large_steering_jump;
        double diameter;
        double forward_noise, perpendicular_noise, velocity_noise;
        std::string base_name;
        std::string axle_joint_name;
        std::string steering_joint_name;
        std::string wheel_link;
        Eigen::Vector3d position;
        double steering_angle, rotation_angle;
        double last_steering_angle, last_rotation_angle;
        double steering_velocity, rotation_velocity;

        bool initialize(void)
        {
            tf::StampedTransform wheel;
            if(!::PoseUKF::lookupTransform(listener, base_name, wheel_link,
                        std::string("Unable to initialize wheel"),
                        &wheel))
                return false;
            tf::vectorTFToEigen(wheel.getOrigin(), position);
            ROS_DEBUG_STREAM("Initialized wheel " << wheel_link << " at " << position.transpose());
            position_initialized = true;
            return true;
        };

        bool delta(double *deltasteer, double *deltarotate)
        {
            *deltasteer = (steering_angle - last_steering_angle);
            *deltarotate = (rotation_angle - last_rotation_angle);
            last_steering_angle = steering_angle;
            last_rotation_angle = rotation_angle;
            if(!angle_initialized)
            {
                angle_initialized = true;
                *deltasteer = 0;
                *deltarotate = 0;
            }
            return (fabs(*deltasteer)<large_steering_jump) &&
                   (fabs(*deltarotate)<large_rotation_jump);
        };
    };

    // storage for all defined wheels
    std::vector<std::shared_ptr<struct Wheel> > wheels_;
    // index into wheels by steering joint
    std::map<std::string, std::shared_ptr<struct Wheel> > steering_values_;
    // index into wheels by rolling joint
    std::map<std::string, std::shared_ptr<struct Wheel> > rotation_values_;

    std::string imu_frame_;
    std::string base_name_;
    std::string odom_frame_id_;
    double publish_period_;
    int seq_;

    void jointStateCallback(sensor_msgs::JointStateConstPtr msg);
    void imuCallback(sensor_msgs::ImuConstPtr msg);
    void gyroCallback(sensor_msgs::ImuConstPtr msg);
    void sendPose(const ros::TimerEvent& e);
    void sendState(void);
    bool lookupTransform(std::string frame_id, std::string to_id, tf::StampedTransform& transform);
    Sophus::SE3d tf2se3(const tf::StampedTransform& transform);

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    ros::Subscriber imu_subscription_;
    ros::Subscriber gyro_subscription_;
    ros::Subscriber joint_subscription_;

    ros::Timer publish_timer_;
    ros::Publisher pose_pub_;
    ros::Publisher state_pub_;
    bool send_on_timer_;

    PoseUKF *ukf_;
    ros::Time last_update_;
    ros::Time last_joint_stamp_;
    ros::Time last_imu_stamp_;

    Eigen::Vector2d sigma_position_;
    Eigen::Vector2d sigma_velocity_;
    Eigen::Vector2d sigma_acceleration_;
    Eigen::Vector3d sigma_orientation_;
    Eigen::Vector3d sigma_omega_;
    Eigen::Vector3d sigma_gyro_bias_;
    Eigen::Vector3d sigma_accel_bias_;

    Eigen::MatrixXd process_noise(double dt) const;
    void parseWheelParameters(const ros::NodeHandle& privatenh);
    void parseProcessSigma(const ros::NodeHandle& privatenh);
    //void parseIMUMeasurementSigma(const ros::NodeHandle& privatenh);
    //void parseOdometryMeasurementSigma(const ros::NodeHandle& privatenh);

    public:
        PoseUKFNode();
        ~PoseUKFNode();
        bool initialize(void);
        void connect(void);
        void printState(void);
};

PoseUKFNode::PoseUKFNode() :
    imu_frame_(""),
    ukf_(NULL)
{
    ros::NodeHandle privatenh("~");
    ros::NodeHandle nh("~");

    privatenh.param("base_name", base_name_, std::string("base_link"));
    privatenh.param("odom_frame_id", odom_frame_id_, std::string("odom"));

    privatenh.param("publish_period", publish_period_, 0.020);

    double alpha, beta, kappa;
    privatenh.param("alpha", alpha, 1e-3);
    privatenh.param("beta", beta, 2.0);
    privatenh.param("kappa", kappa, 0.0);

    parseWheelParameters(privatenh);

    ukf_ = new PoseUKF(alpha, beta, kappa);

    std::vector<double> cov_vect;
    Eigen::MatrixXd initial_covariance;
    if(privatenh.getParam("initial_covariance", cov_vect))
    {
        int ndim = PoseState().ndim();
        PoseState st = ukf_->state();
        st.Covariance = Eigen::MatrixXd::Map(cov_vect.data(), ndim, ndim);
        ukf_->reset(st);
    }

    parseProcessSigma(privatenh);
    //parseIMUMeasurementSigma(privatenh);
    //parseOdometryMeasurementSigma(privatenh);

    privatenh.param("send_on_timer", send_on_timer_, true);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
    state_pub_ = privatenh.advertise<pose_ukf::Pose>("filter_state", 1);
    seq_ = 0;
    if(send_on_timer_)
    {
        publish_timer_ = privatenh.createTimer(ros::Duration(publish_period_), &PoseUKFNode::sendPose, this);
    }
}

void
PoseUKFNode::connect(void)
{
    ros::NodeHandle nh;
    last_update_ = ros::Time::now();
    last_joint_stamp_ = last_update_;
    last_imu_stamp_ = last_update_;
    imu_subscription_ = nh.subscribe("imu", 1, &PoseUKFNode::imuCallback, this);
    gyro_subscription_ = nh.subscribe("gyro", 1, &PoseUKFNode::gyroCallback, this);
    joint_subscription_ = nh.subscribe("joint_state", 1, &PoseUKFNode::jointStateCallback, this);
    ROS_INFO("Subscribers created");
    publish_timer_.start();
}

PoseUKFNode::~PoseUKFNode()
{
}

bool
PoseUKFNode::initialize(void)
{
    bool success = true;
    for(auto &w : wheels_)
    {
        success &= w->initialize();
        if(!success)
            break;
    }
    return success;
}

void
PoseUKFNode::sendPose(const ros::TimerEvent& e)
{
    tf::StampedTransform imu_transform;
    if(!lookupTransform(imu_frame_, base_name_, imu_transform))
    {
        return;
    }

    (void)e;
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped());

    msg->header.frame_id = odom_frame_id_;
    msg->header.stamp = last_update_;
    msg->header.seq = seq_++;

    tf::Quaternion q;
    tf::quaternionEigenToTF(ukf_->state().Orientation.unit_quaternion(), q);
    tf::Vector3 v;
    Eigen::Vector3d pos3d;
    pos3d.segment<2>(0) = ukf_->state().Position;
    pos3d(2) = 0;
    tf::vectorEigenToTF(pos3d, v);
    tf::Pose imu_tf(q, v);
    tf::Pose odom_tf = imu_transform * imu_tf * imu_transform.inverse();
    tf::poseTFToMsg( odom_tf, msg->pose);
    pose_pub_.publish(msg);
}

void
PoseUKFNode::sendState(void)
{
    pose_ukf::PosePtr msg(new pose_ukf::Pose);

    msg->header.stamp = last_update_;
    msg->header.frame_id = imu_frame_;

    msg->Position.x = ukf_->state().Position(0);
    msg->Position.y = ukf_->state().Position(1);
    msg->Velocity.x = ukf_->state().Velocity(0);
    msg->Velocity.y = ukf_->state().Velocity(1);
    msg->Acceleration.x = ukf_->state().Acceleration(0);
    msg->Acceleration.y = ukf_->state().Acceleration(1);
    Eigen::Quaterniond q=ukf_->state().Orientation.unit_quaternion();
    tf::quaternionEigenToMsg(q, msg->Orientation);
    tf::vectorEigenToMsg(ukf_->state().Omega, msg->Omega);
    tf::vectorEigenToMsg(ukf_->state().AccelBias, msg->AccelBias);
    tf::vectorEigenToMsg(ukf_->state().GyroBias, msg->GyroBias);

    double qn = q.norm();
    state_pub_.publish(msg);
    //printState();
}

Eigen::MatrixXd
PoseUKFNode::process_noise(double dt) const
{
    Eigen::MatrixXd noise(ukf_->ndim(), ukf_->ndim());
    noise.setZero();

    // position, velocity, orientation, omega, gyro_bias, accel_bias
    noise.block<2,2>(0,0).diagonal() = (dt*sigma_position_).cwiseProduct(dt*sigma_position_);
    noise.block<2,2>(0,2).diagonal() = (dt*dt*sigma_velocity_/2.).cwiseProduct(dt*dt*sigma_velocity_/2.);
    noise.block<2,2>(2,0).diagonal() = (dt*dt*sigma_velocity_/2.).cwiseProduct(dt*dt*sigma_velocity_/2.);
    noise.block<2,2>(2,2).diagonal() = (dt*sigma_velocity_).cwiseProduct(dt*sigma_velocity_);
    noise.block<2,2>(2,4).diagonal() = (dt*dt*sigma_acceleration_/2.).cwiseProduct(dt*dt*sigma_acceleration_/2.);
    noise.block<2,2>(4,2).diagonal() = (dt*dt*sigma_acceleration_/2.).cwiseProduct(dt*dt*sigma_acceleration_/2.);
    noise.block<2,2>(0,4).diagonal() = (dt*dt*dt*sigma_acceleration_/3.).cwiseProduct(dt*dt*dt*sigma_acceleration_/3.);
    noise.block<2,2>(4,0).diagonal() = (dt*dt*dt*sigma_acceleration_/3.).cwiseProduct(dt*dt*dt*sigma_acceleration_/3.);
    noise.block<2,2>(4,4).diagonal() = (dt*sigma_acceleration_).cwiseProduct(dt*sigma_acceleration_);
    noise.block<3,3>(6,6).diagonal() = (dt*sigma_orientation_).cwiseProduct(dt*sigma_orientation_);
    noise.block<3,3>(6,9).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
    noise.block<3,3>(9,6).diagonal() = (dt*dt*sigma_omega_/2.).cwiseProduct(dt*dt*sigma_omega_/2.);
    noise.block<3,3>(9,9).diagonal() = (dt*sigma_omega_).cwiseProduct(dt*sigma_omega_);
    noise.block<3,3>(12,12).diagonal() = (dt*sigma_gyro_bias_).cwiseProduct(dt*sigma_gyro_bias_);
    noise.block<3,3>(15,15).diagonal() = (dt*sigma_accel_bias_).cwiseProduct(dt*sigma_accel_bias_);

    return noise;
}

void
PoseUKFNode::imuCallback(sensor_msgs::ImuConstPtr msg)
{
    imu_frame_ = msg->header.frame_id;
    IMUOrientationMeasurement m;
    tf::vectorMsgToEigen(msg->linear_acceleration,
            m.acceleration);
    tf::vectorMsgToEigen(msg->angular_velocity,
            m.omega);
    Eigen::MatrixXd meas_cov(m.ndim(), m.ndim());
    meas_cov.setZero();
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > accel_cov(msg->linear_acceleration_covariance.data());
    meas_cov.block<3, 3>(0, 0) = accel_cov;
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > omega_cov(msg->angular_velocity_covariance.data());
    meas_cov.block<3, 3>(3, 3) = omega_cov;
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>0)
    {
        ROS_DEBUG_STREAM("Performing IMU predict with dt=" << dt );
        ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
        ukf_->predict(dt, process_noise(dt));
        ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
        last_update_ = msg->header.stamp;
    }
    else
    {
        ROS_DEBUG_STREAM("Skipping imu predict dt=" << dt);
    }

    ROS_DEBUG_STREAM("IMU correct:\n" << m);
    ROS_DEBUG_STREAM("IMU measurement covariance:\n" << meas_cov);
    ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
    ukf_->correct(m, meas_covs);
    ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
    sendState();
    if(!send_on_timer_)
    {
        ros::TimerEvent e;
        sendPose(e);
    }
}


bool PoseUKFNode::lookupTransform(std::string frame_id, std::string to_id, tf::StampedTransform& transform)
{
    if(frame_id=="" || to_id=="")
    {
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

Sophus::SE3d PoseUKFNode::tf2se3(const tf::StampedTransform& transform)
{
    Sophus::SE3d se3;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    tf::quaternionTFToEigen(transform.getRotation(), rotation);
    tf::vectorTFToEigen(transform.getOrigin(), translation);
    se3 = Sophus::SE3d(rotation, translation);
    return se3;
}

void
PoseUKFNode::gyroCallback(sensor_msgs::ImuConstPtr msg)
{
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
    tf::quaternionTFToEigen(gyro_imu.getRotation(), imu_gq);
    YawMeasurement<PoseState> m;
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
    if(!send_on_timer_)
    {
        ros::TimerEvent e;
        sendPose(e);
    }
}

void
PoseUKFNode::printState(void)
{
    ROS_INFO_STREAM("State:\n" << ukf_->state());
    Eigen::MatrixXd cov = ukf_->state().Covariance;
    ROS_INFO_STREAM("position cov:\n" << (cov.block<2,2>(0,0)));
    ROS_INFO_STREAM("velocity cov:\n" << (cov.block<2,2>(2,2)));
    ROS_INFO_STREAM("orientation cov:\n" << (cov.block<3,3>(4,4)));
    ROS_INFO_STREAM("omega cov:\n" << (cov.block<3,3>(7,7)));
    ROS_INFO_STREAM("gyro bias cov:\n" << (cov.block<3,3>(10,10)));
    ROS_INFO_STREAM("accel bias cov:\n" << (cov.block<3,3>(13,13)));
    ROS_INFO_STREAM("covariance:\n" << cov);
}

void
PoseUKFNode::jointStateCallback(sensor_msgs::JointStateConstPtr msg)
{
    tf::StampedTransform imu_transform;
    if(!lookupTransform(msg->header.frame_id, imu_frame_, imu_transform))
    {
        return;
    }

    Sophus::SE3d imu_se3 = tf2se3(imu_transform);

    for( auto&& t: zip_range(msg->name, msg->position, msg->velocity) )
    {
        std::string name = boost::get<0>(t);
        double position = boost::get<1>(t);
        double velocity = boost::get<2>(t);
        if( steering_values_.find(name) != steering_values_.end() )
        {
            steering_values_[name]->steering_angle = position;
            steering_values_[name]->steering_velocity = velocity;
        }
        if( rotation_values_.find(name) != rotation_values_.end() )
        {
            rotation_values_[name]->rotation_angle = position;
            rotation_values_[name]->rotation_velocity = velocity;
        }
    }

    bool good_delta=true;
    Eigen::VectorXd wheel_deltas(2*wheels_.size());
    Eigen::VectorXd wheel_velocities(2*wheels_.size());
    Eigen::MatrixXd shape(2*wheels_.size(), 3);
    Eigen::Vector2d direction;
    Eigen::Matrix2d cross;
    cross << 0, -1,
             1,  0;
    Eigen::Vector2d R;
    Eigen::MatrixXd measurement_cov(2*wheels_.size(), 2*wheels_.size());
    measurement_cov.setZero();
    for(size_t i=0;i<wheels_.size();i++)
    {
        double dphi, dtheta;
        good_delta &= wheels_[i]->delta(&dphi, &dtheta);
        direction << cos(wheels_[i]->steering_angle),
                     sin(wheels_[i]->steering_angle);
        wheel_deltas.segment<2>(2*i) = wheels_[i]->diameter*dtheta*direction/2.;
        wheel_velocities.segment<2>(2*i) = wheels_[i]->diameter*wheels_[i]->rotation_velocity*direction/2.;
        R = wheels_[i]->position.segment<2>(0);
        shape.block<2,1>(2*i,0) = cross * R;
        shape.block<2,2>(2*i,1) = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d wheel_sigma;
        wheel_sigma << wheels_[i]->forward_noise, 0,
                       0, wheels_[i]->perpendicular_noise;
        Eigen::Matrix2d rcov = Eigen::Rotation2Dd(wheels_[i]->steering_angle).toRotationMatrix();
        measurement_cov.block<2,2>(2*i, 2*i) = rcov*wheel_sigma*wheel_sigma.transpose()*rcov.transpose();
    }
    if(!good_delta)
    {
        ROS_ERROR("Large angle jump, skipping odometry update");
        return;
    }

    if(!(((measurement_cov-measurement_cov).array() == (measurement_cov-measurement_cov).array()).all()))
    {
        ROS_ERROR_STREAM("Covariance not finite\n" << measurement_cov );
        return;
    }

    Eigen::Matrix<double, 3, 6> solution = (shape.transpose()*shape).inverse()*shape.transpose();
    Eigen::Vector3d motion = solution*wheel_deltas;
    Eigen::Vector3d velocity = solution*wheel_velocities;
    Eigen::Matrix3d covariance = (shape.transpose()*measurement_cov.inverse()*shape).inverse();

    if(!(((covariance-covariance).array() == (covariance-covariance).array()).all()))
    {
        ROS_ERROR_STREAM("Rotated covariance not finite\n" << covariance );
        return;
    }

    Sophus::SE3d odometry_delta_base_link;
    odometry_delta_base_link.translation() << motion(1), motion(2), 0;
    odometry_delta_base_link.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, motion(0)));
    Sophus::SE3d odometry_delta_imu = imu_se3*odometry_delta_base_link*imu_se3.inverse();

    Sophus::SE3d odometry_velocity_base_link;
    odometry_velocity_base_link.translation() << velocity(1), velocity(2), 0;
    odometry_velocity_base_link.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, velocity(0)));
    Sophus::SE3d odometry_velocity_imu = imu_se3*odometry_velocity_base_link*imu_se3.inverse();

    Eigen::Matrix3d odometry_position_covariance_base_link;
    odometry_position_covariance_base_link.setIdentity();
    odometry_position_covariance_base_link.block<2,2>(0,0) = covariance.block<2,2>(1,1);
    Eigen::Matrix3d rot_base_to_imu;
    tf::matrixTFToEigen(imu_transform.getBasis(), rot_base_to_imu);
    Eigen::Matrix3d odometry_position_covariance_imu = rot_base_to_imu*odometry_position_covariance_base_link*rot_base_to_imu.transpose();

    double delta_t = (msg->header.stamp - last_joint_stamp_).toSec();
    last_joint_stamp_ = msg->header.stamp;

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>0)
    {
        ROS_DEBUG_STREAM("Performing odometry predict with dt=" << dt);
        ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
        ukf_->predict(dt, process_noise(dt));
        ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
        last_update_ = msg->header.stamp;
    }
    else
    {
        ROS_DEBUG_STREAM("Skipping odometry predict with dt=" << dt);
    }

    WheelOdometryMeasurement m;
    m.delta_yaw = odometry_delta_imu.so3().log()(2);
    m.yaw_rate = odometry_velocity_imu.so3().log()(2);
    m.delta_pos = odometry_delta_imu.translation().segment<2>(0);
    m.velocity = odometry_velocity_imu.translation().segment<2>(0);
    std::vector<Eigen::MatrixXd> meas_covs;
    Eigen::MatrixXd meas_cov;
    meas_cov.resize(m.ndim(), m.ndim());
    meas_cov.setZero();
    meas_cov(0,0) = covariance(0,0);
    meas_cov(1,1) = covariance(0,0)/delta_t/delta_t;
    meas_cov.block<2,2>(2,2) = odometry_position_covariance_imu.block<2,2>(0,0);
    meas_cov.block<2,2>(4,4) = odometry_position_covariance_imu.block<2,2>(0,0)/delta_t/delta_t;
    meas_covs.push_back(meas_cov);
    ROS_DEBUG_STREAM("odometry correct:\n" << m);
    //ROS_DEBUG_STREAM("odometry prediction:\n" << m.measure(ukf_->state(), Eigen::VectorXd::Zero(m.ndim())));
    ROS_DEBUG_STREAM("odometry measurement covariance:\n" << meas_cov);
    ukf_->differentialcorrect(m, meas_covs);
    ROS_DEBUG_STREAM("AccelBias: " << ukf_->state().AccelBias.transpose());
    sendState();
}

void
PoseUKFNode::parseWheelParameters(const ros::NodeHandle& privatenh)
{
    double wheel_forward_noise, wheel_perpendicular_noise, wheel_velocity_noise;
    double large_rotation_jump, large_steering_jump;
    double wheel_diameter;

    // retrieve default wheel parameters
    privatenh.param("wheel_forward_noise", wheel_forward_noise, 0.002);
    privatenh.param("wheel_perpendicular_noise", wheel_perpendicular_noise, 0.001);
    privatenh.param("wheel_velocity_noise", wheel_velocity_noise, 0.001);
    privatenh.param("wheel_diameter", wheel_diameter, 0.13);
    privatenh.param("large_rotation_jump", large_rotation_jump, 1024.0);
    privatenh.param("large_steering_jump", large_steering_jump, 1024.0);
    // parse any list of wheels
    XmlRpc::XmlRpcValue wheel_list;
    if(!privatenh.getParam("wheels", wheel_list))
    {
        ROS_ERROR("No wheels defined, wheel odometry not available");
        return;
    }
    ROS_ASSERT(wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int wi=0; wi<wheel_list.size(); wi++)
    {
        XmlRpc::XmlRpcValue wheel_parameters = wheel_list[wi];
        ROS_ASSERT(wheel_parameters.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        std::shared_ptr<struct Wheel> w(new struct Wheel(wheel_diameter,
                    wheel_forward_noise, wheel_perpendicular_noise,
                    wheel_velocity_noise,
                    large_rotation_jump, large_steering_jump,
                    base_name_));
        bool have_axle=false, have_steering=false, have_wheel=false;
        if(wheel_parameters.hasMember(std::string("large_rotation_jump")))
        {
            XmlRpc::XmlRpcValue jump = wheel_parameters[std::string("large_rotation_jump")];
            ROS_ASSERT(jump.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                       jump.getType() == XmlRpc::XmlRpcValue::TypeInt);
            w->large_rotation_jump = double(jump);
        }
        if(wheel_parameters.hasMember(std::string("large_steering_jump")))
        {
            XmlRpc::XmlRpcValue jump = wheel_parameters[std::string("large_steering_jump")];
            ROS_ASSERT(jump.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                       jump.getType() == XmlRpc::XmlRpcValue::TypeInt);
            w->large_steering_jump = double(jump);
        }
        if(wheel_parameters.hasMember(std::string("diameter")))
        {
            XmlRpc::XmlRpcValue diam = wheel_parameters[std::string("diameter")];
            ROS_ASSERT(diam.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                       diam.getType() == XmlRpc::XmlRpcValue::TypeInt);
            w->diameter = double(diam);
        }
        if(wheel_parameters.hasMember(std::string("forward_noise")))
        {
            XmlRpc::XmlRpcValue noise = wheel_parameters[std::string("forward_noise")];
            ROS_ASSERT(noise.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                       noise.getType() == XmlRpc::XmlRpcValue::TypeInt);
            w->forward_noise = double(noise);
        }
        if(wheel_parameters.hasMember(std::string("perpendicular_noise")))
        {
            XmlRpc::XmlRpcValue noise = wheel_parameters[std::string("perpendicular_noise")];
            ROS_ASSERT(noise.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                       noise.getType() == XmlRpc::XmlRpcValue::TypeInt);
            w->perpendicular_noise = double(noise);
        }
        if(wheel_parameters.hasMember(std::string("velocity_noise")))
        {
            XmlRpc::XmlRpcValue noise = wheel_parameters[std::string("velocity_noise")];
            ROS_ASSERT(noise.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                       noise.getType() == XmlRpc::XmlRpcValue::TypeInt);
            w->velocity_noise = double(noise);
        }
        if(wheel_parameters.hasMember(std::string("axle_joint_name")))
        {
            XmlRpc::XmlRpcValue name = wheel_parameters[std::string("axle_joint_name")];
            ROS_ASSERT(name.getType() == XmlRpc::XmlRpcValue::TypeString);
            w->axle_joint_name = std::string(name);
            have_axle = true;
        }
        if(wheel_parameters.hasMember(std::string("steering_joint_name")))
        {
            XmlRpc::XmlRpcValue name = wheel_parameters[std::string("steering_joint_name")];
            ROS_ASSERT(name.getType() == XmlRpc::XmlRpcValue::TypeString);
            w->steering_joint_name = std::string(name);
            have_steering = true;
        }
        if(wheel_parameters.hasMember(std::string("wheel_link")))
        {
            XmlRpc::XmlRpcValue name = wheel_parameters[std::string("wheel_link")];
            ROS_ASSERT(name.getType() == XmlRpc::XmlRpcValue::TypeString);
            w->wheel_link = std::string(name);
            have_wheel = true;
        }
        if(have_axle && have_steering && have_wheel)
        {
            wheels_.push_back(w);
            steering_values_[w->steering_joint_name] = w;
            rotation_values_[w->axle_joint_name] = w;
            ROS_DEBUG_STREAM("Added wheel:");
            ROS_DEBUG_STREAM("   diameter            : " << w->diameter);
            ROS_DEBUG_STREAM("   wheel link          : " << w->wheel_link);
            ROS_DEBUG_STREAM("   steering joint      : " << w->steering_joint_name);
            ROS_DEBUG_STREAM("   axle joint          : " << w->axle_joint_name);
            ROS_DEBUG_STREAM("   forward noise       : " << w->forward_noise);
            ROS_DEBUG_STREAM("   perpendicular noise : " << w->perpendicular_noise);
        }
        else
        {
            if(!have_steering)
                ROS_ERROR("Wheel with no steering joint ignored");
            if(!have_wheel)
                ROS_ERROR("Wheel with no wheel link ignored");
            if(!have_axle)
                ROS_ERROR("Wheel with no axle joint ignored");
        }
    }
}

template<int N>
void listToVec(XmlRpc::XmlRpcValue& list, Eigen::Matrix<double, N, 1> *vec)
{
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i<std::min(N, list.size()); i++)
    {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        (*vec)(i) = static_cast<double>(list[i]);
    }
}

void
PoseUKFNode::parseProcessSigma(const ros::NodeHandle& privatenh)
{
    if(!privatenh.hasParam("process_sigma"))
    {
        ROS_ERROR("No process noise sigma specified");
        exit(1);
    }
    XmlRpc::XmlRpcValue process_sigma;
    privatenh.getParam("process_sigma", process_sigma);
    ROS_ASSERT(process_sigma.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(process_sigma.hasMember(std::string("position")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("position")];
        listToVec(list, &sigma_position_);
    }
    if(process_sigma.hasMember(std::string("velocity")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("velocity")];
        listToVec(list, &sigma_velocity_);
    }
    if(process_sigma.hasMember(std::string("acceleration")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("acceleration")];
        listToVec(list, &sigma_acceleration_);
    }
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
    ros::init(argc, argv, "PoseUKF");
    PoseUKF::PoseUKFNode n;
    while(!n.initialize() && !ros::isShuttingDown())
        ros::Duration(1.0).sleep();
    if(!ros::isShuttingDown())
    {
        n.connect();
        ros::spin();
    }
}
