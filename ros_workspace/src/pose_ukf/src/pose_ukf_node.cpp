#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pose_ukf/pose_ukf.hpp>
#include <pose_ukf/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <functional>
#include <sophus/se3.hpp>
#include <nav_msgs/Odometry.h>


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
            if(!lookupTransform(listener, base_name, wheel_link,
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

    double wheel_forward_noise_, wheel_perpendicular_noise_, wheel_velocity_noise_;
    double large_rotation_jump_, large_steering_jump_;
    double wheel_diameter_;
    std::string base_name_;
    double publish_period_;
    int seq_;

    std::vector<std::shared_ptr<struct Wheel> > wheels_;
    std::map<std::string, std::shared_ptr<struct Wheel> > steering_values_;
    std::map<std::string, std::shared_ptr<struct Wheel> > rotation_values_;

    void jointStateCallback(sensor_msgs::JointStateConstPtr msg);
    void imuCallback(sensor_msgs::ImuConstPtr msg);
    void visualOdometryCallback(nav_msgs::OdometryConstPtr msg);
    void sendPose(const ros::TimerEvent& e);
    void sendState(void);

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    ros::Subscriber imu_subscription_;
    ros::Subscriber joint_subscription_;
    ros::Subscriber visual_odometry_subscription_;

    ros::Timer publish_timer_;
    ros::Publisher pose_pub_;
    ros::Publisher state_pub_;

    tf::StampedTransform imu_base_transform_;
    Sophus::SE3d imu_base_se3_;
    bool have_imu_base_transform_;

    tf::StampedTransform imu_camera_transform_;
    Sophus::SE3d imu_camera_se3_;
    bool have_imu_camera_transform_;

    PoseUKF *ukf_;
    ros::Time last_update_;

    ros::Time last_joint_stamp_;
    struct PoseState last_joint_state_;
    Eigen::MatrixXd last_joint_covariance_;

    ros::Time last_imu_stamp_;

    nav_msgs::Odometry last_visual_odom_;
    struct PoseState last_visual_state_;
    Eigen::MatrixXd last_visual_covariance_;

    Eigen::Vector2d sigma_position_;
    Eigen::Vector2d sigma_velocity_;
    Eigen::Vector2d sigma_acceleration_;
    Eigen::Vector3d sigma_orientation_;
    Eigen::Vector3d sigma_omega_;
    Eigen::Vector3d sigma_gyro_bias_;
    Eigen::Vector3d sigma_accel_bias_;

    bool use_vo_covariance_;
    Eigen::Vector3d visual_position_sigma_;
    Eigen::Vector3d visual_velocity_sigma_;
    Eigen::Vector3d visual_orientation_sigma_;
    Eigen::Vector3d visual_omega_sigma_;

    Eigen::MatrixXd process_noise(double dt) const;
    void parseWheelParameters(const ros::NodeHandle& privatenh);
    void parseProcessSigma(const ros::NodeHandle& privatenh);
    //void parseIMUMeasurementSigma(const ros::NodeHandle& privatenh);
    //void parseOdometryMeasurementSigma(const ros::NodeHandle& privatenh);
    void parseVisualMeasurementSigma(const ros::NodeHandle& privatenh);

    public:
        PoseUKFNode();
        ~PoseUKFNode();
        bool initialize(void);
        void connect(void);
        void printState(void);
};

PoseUKFNode::PoseUKFNode() :
    have_imu_base_transform_(false),
    have_imu_camera_transform_(false),
    ukf_(NULL),
    use_vo_covariance_(false)
{
    ros::NodeHandle privatenh("~");
    ros::NodeHandle nh("~");

    privatenh.param("base_name", base_name_, std::string("base_link"));

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
        initial_covariance = Eigen::MatrixXd::Map(cov_vect.data(), ndim, ndim);
        PoseState st = ukf_->state();
        ukf_->reset(st, initial_covariance);
    }

    parseProcessSigma(privatenh);
    //parseIMUMeasurementSigma(privatenh);
    //parseOdometryMeasurementSigma(privatenh);
    parseVisualMeasurementSigma(privatenh);

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
    state_pub_ = privatenh.advertise<pose_ukf::Pose>("filter_state", 1);
    seq_ = 0;
    publish_timer_ = privatenh.createTimer(ros::Duration(publish_period_), &PoseUKFNode::sendPose, this);
}

void
PoseUKFNode::connect(void)
{
    ros::NodeHandle nh;
    last_update_ = ros::Time::now();

    last_joint_stamp_ = last_update_;
    last_joint_state_ = ukf_->state();
    last_joint_covariance_ = ukf_->covariance();

    last_imu_stamp_ = last_update_;

    last_visual_odom_.header.stamp = ros::Time(0);

    imu_subscription_ = nh.subscribe("imu", 1, &PoseUKFNode::imuCallback, this);
    joint_subscription_ = nh.subscribe("joint_state", 1, &PoseUKFNode::jointStateCallback, this);
    visual_odometry_subscription_ = nh.subscribe("visual_odometry", 1, &PoseUKFNode::visualOdometryCallback, this);
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
    if(!have_imu_base_transform_)
        return;

    (void)e;
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped());

    msg->header.frame_id = imu_base_transform_.frame_id_;
    msg->header.stamp = last_update_;
    msg->header.seq = seq_++;
    Eigen::Vector3d pos3d;
    pos3d.segment<2>(0) = ukf_->state().Position;
    pos3d(2) = 0;
    tf::pointEigenToMsg(pos3d, msg->pose.position);
    tf::quaternionEigenToMsg(ukf_->state().Orientation.unit_quaternion(),
                             msg->pose.orientation);
    pose_pub_.publish(msg);
}

void
PoseUKFNode::sendState(void)
{
    pose_ukf::PosePtr msg(new pose_ukf::Pose);

    msg->header.stamp = last_update_;
    msg->header.frame_id = imu_base_transform_.frame_id_;

    msg->Position.x = ukf_->state().Position(0);
    msg->Position.y = ukf_->state().Position(1);
    msg->Velocity.x = ukf_->state().Velocity(0);
    msg->Velocity.y = ukf_->state().Velocity(1);
    msg->Acceleration.x = ukf_->state().Acceleration(0);
    msg->Acceleration.y = ukf_->state().Acceleration(1);
    tf::quaternionEigenToMsg(ukf_->state().Orientation.unit_quaternion(), msg->Orientation);
    tf::vectorEigenToMsg(ukf_->state().Omega, msg->Omega);
    tf::vectorEigenToMsg(ukf_->state().AccelBias, msg->AccelBias);
    tf::vectorEigenToMsg(ukf_->state().GyroBias, msg->GyroBias);

    state_pub_.publish(msg);
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

    if(!have_imu_base_transform_)
    {
        try
        {
            listener_.lookupTransform(msg->header.frame_id,
                    base_name_,
                    ros::Time(0),
                    imu_base_transform_);
            Eigen::Quaterniond imu_rotation;
            Eigen::Vector3d imu_translation;
            tf::quaternionTFToEigen(imu_base_transform_.getRotation(), imu_rotation);
            tf::vectorTFToEigen(imu_base_transform_.getOrigin(), imu_translation);
            imu_base_se3_ = Sophus::SE3d(imu_rotation, imu_translation);
            have_imu_base_transform_ = true;
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR_STREAM("Unable to look up imu frame for wheel odometry: " << ex.what());
            return;
        }
    }

    IMUOrientationMeasurement m;
    tf::vectorMsgToEigen(msg->linear_acceleration,
            m.acceleration);
    m.acceleration *= m.littleg;
    tf::vectorMsgToEigen(msg->angular_velocity,
            m.omega);
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
        ROS_DEBUG_STREAM("Performing IMU predict with dt=" << dt );
        ROS_DEBUG_STREAM("Process noise:\n" << process_noise(dt));
        ukf_->predict(dt, process_noise(dt));
        last_update_ = msg->header.stamp;
    }

    ROS_DEBUG_STREAM("IMU correct:\n" << m);
    ROS_DEBUG_STREAM("IMU measurement covariance:\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    //printState();
    sendState();
}

void
PoseUKFNode::printState(void)
{
    ROS_INFO_STREAM("State:\n" << ukf_->state());
    ROS_INFO_STREAM("position cov:\n" << (ukf_->covariance().block<2,2>(0,0)));
    ROS_INFO_STREAM("velocity cov:\n" << (ukf_->covariance().block<2,2>(2,2)));
    ROS_INFO_STREAM("acceleration cov:\n" << (ukf_->covariance().block<2,2>(4,4)));
    ROS_INFO_STREAM("orientation cov:\n" << (ukf_->covariance().block<3,3>(6,6)));
    ROS_INFO_STREAM("omega cov:\n" << (ukf_->covariance().block<3,3>(9,9)));
    ROS_INFO_STREAM("gyro bias cov:\n" << (ukf_->covariance().block<3,3>(12,12)));
    ROS_INFO_STREAM("accel bias cov:\n" << (ukf_->covariance().block<3,3>(15,15)));
    ROS_INFO_STREAM("covariance:\n" << ukf_->covariance());
}

void
PoseUKFNode::jointStateCallback(sensor_msgs::JointStateConstPtr msg)
{
    if(!have_imu_base_transform_ || wheels_.size() == 0)
        return;

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
        wheel_deltas.segment<2>(2*i) = wheels_[i]->diameter*dtheta*direction;
        wheel_velocities.segment<2>(2*i) = wheels_[i]->diameter*wheels_[i]->rotation_velocity*direction;
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
    Sophus::SE3d odometry_delta_imu = imu_base_se3_*odometry_delta_base_link*imu_base_se3_.inverse();

    Sophus::SE3d odometry_velocity_base_link;
    odometry_velocity_base_link.translation() << velocity(1), velocity(2), 0;
    odometry_velocity_base_link.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, velocity(0)));
    Sophus::SE3d odometry_velocity_imu = imu_base_se3_*odometry_velocity_base_link*imu_base_se3_.inverse();

    Eigen::Matrix3d odometry_position_covariance_base_link;
    odometry_position_covariance_base_link.setIdentity();
    odometry_position_covariance_base_link.block<2,2>(0,0) = covariance.block<2,2>(1,1);
    Eigen::Matrix3d rot_base_to_imu;
    tf::matrixTFToEigen(imu_base_transform_.getBasis(), rot_base_to_imu);
    Eigen::Matrix3d odometry_position_covariance_imu = rot_base_to_imu*odometry_position_covariance_base_link*rot_base_to_imu.transpose();

    double delta_t = (msg->header.stamp - last_joint_stamp_).toSec();
    last_joint_stamp_ = msg->header.stamp;

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>0)
    {
        ROS_DEBUG_STREAM("Performing odometry predict with dt=" << dt);
        ROS_DEBUG_STREAM("Process noise:\n" << process_noise(dt));
        ukf_->predict(dt, process_noise(dt));
        last_update_ = msg->header.stamp;
    }

    WheelOdometryMeasurement m;
    m.delta_yaw = odometry_delta_imu.so3().log()(2);
    m.yaw_rate = odometry_velocity_imu.so3().log()(2);
    m.delta_pos = odometry_delta_imu.translation().segment<2>(0);
    m.velocity = odometry_velocity_imu.translation().segment<2>(0);
    m.last_state = last_joint_state_;
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
    ROS_DEBUG_STREAM("odometry measurement covariance:\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    last_joint_state_ = ukf_->state();
    last_joint_covariance_ = ukf_->covariance();
    //printState();
    sendState();
}

void
PoseUKFNode::visualOdometryCallback(nav_msgs::OdometryConstPtr msg)
{
    if(!have_imu_base_transform_)
        return;

    if(!have_imu_camera_transform_)
    {
        try
        {
            listener_.lookupTransform(msg->header.frame_id,
                    imu_base_transform_.child_frame_id_,
                    ros::Time(0),
                    imu_camera_transform_);
            Eigen::Quaterniond imu_rotation;
            Eigen::Vector3d imu_translation;
            tf::quaternionTFToEigen(imu_camera_transform_.getRotation(), imu_rotation);
            tf::vectorTFToEigen(imu_camera_transform_.getOrigin(), imu_translation);
            imu_camera_se3_ = Sophus::SE3d(imu_rotation, imu_translation);
            have_imu_camera_transform_ = true;
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR_STREAM("Unable to look up imu frame for VO: " << ex.what());
            return;
        }
    }

    if(last_visual_odom_.header.stamp == ros::Time(0))
    {
        last_visual_odom_ = *msg;
        last_visual_state_ = ukf_->state();
        last_visual_covariance_ = ukf_->covariance();

        ROS_INFO_STREAM("First VO message at " << msg->header.stamp << ", Initializing.");
        return;
    }

    struct VisualOdometryMeasurement m;
    m.last_state = last_visual_state_;

    Eigen::Quaterniond last_vo_q, this_vo_q;
    tf::quaternionMsgToEigen(last_visual_odom_.pose.pose.orientation, last_vo_q);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, this_vo_q);
    Eigen::Vector3d last_vo_position, this_vo_position;
    tf::pointMsgToEigen(last_visual_odom_.pose.pose.position, last_vo_position);
    tf::pointMsgToEigen(msg->pose.pose.position, this_vo_position);
    Sophus::SE3d imu_delta = imu_camera_se3_*Sophus::SE3d(this_vo_q*last_vo_q.inverse(),
                this_vo_position - last_vo_position)*imu_camera_se3_.inverse();
    m.delta_pos = imu_delta.translation().segment<2>(0);
    m.delta_orientation = imu_delta.so3();

    Eigen::Vector3d vo_velocity_camera, vo_omega_camera;
    tf::vectorMsgToEigen(msg->twist.twist.angular, vo_omega_camera);
    tf::vectorMsgToEigen(msg->twist.twist.linear, vo_velocity_camera);
    Sophus::SE3d vo_velocity_imu = \
                           imu_camera_se3_*\
                           Sophus::SE3d(Sophus::SO3d::exp(vo_omega_camera),
                                        vo_velocity_camera)*\
                           imu_camera_se3_.inverse();
    m.omega = vo_velocity_imu.so3().log();
    m.velocity = vo_velocity_imu.translation().segment<2>(0);

    Eigen::MatrixXd meas_cov;
    meas_cov.resize(m.ndim(), m.ndim());
    meas_cov.setZero();
    Eigen::Matrix3d rot_camera_to_imu;
    tf::matrixTFToEigen(imu_camera_transform_.getBasis(), rot_camera_to_imu);
    Eigen::Matrix<double, 12, 12> cov_rot;
    cov_rot << rot_camera_to_imu, rot_camera_to_imu,
               rot_camera_to_imu, rot_camera_to_imu;
    Eigen::MatrixXd vo_cov_rot;
    double delta_t = (msg->header.stamp - last_visual_odom_.header.stamp).toSec();
    if(use_vo_covariance_)
    {
        Eigen::Matrix<double, 12, 12> msg_cov;
        msg_cov.block<6,6>(0,0) = Eigen::Matrix<double, 6,6>::Map(msg->pose.covariance.data());
        msg_cov.block<6,6>(6,6) = Eigen::Matrix<double, 6,6>::Map(msg->twist.covariance.data());
        msg_cov.block<3,3>(3,3).swap(msg_cov.block<3,3>(6,6));
        msg_cov.block<3,3>(0,3).swap(msg_cov.block<3,3>(0,6));
        msg_cov.block<3,3>(3,0).swap(msg_cov.block<3,3>(6,0));
        msg_cov.block<3,3>(6,9).swap(msg_cov.block<3,3>(3,9));
        msg_cov.block<3,3>(9,6).swap(msg_cov.block<3,3>(9,3));
        vo_cov_rot = cov_rot*msg_cov*cov_rot.transpose();
    }
    else
    {
        Eigen::MatrixXd vo_cov(4*3, 4*3);
        vo_cov.block<3,3>(0,0).diagonal() = (delta_t*visual_position_sigma_).cwiseProduct(delta_t*visual_position_sigma_);
        vo_cov.block<3,3>(3,3).diagonal() = (delta_t*visual_velocity_sigma_).cwiseProduct(delta_t*visual_velocity_sigma_);
        vo_cov.block<3,3>(0,3).diagonal() = (delta_t*delta_t/2.0*visual_velocity_sigma_).cwiseProduct(delta_t*delta_t/2.0*visual_velocity_sigma_);
        vo_cov.block<3,3>(3,0).diagonal() = (delta_t*delta_t/2.0*visual_velocity_sigma_).cwiseProduct(delta_t*delta_t/2.0*visual_velocity_sigma_);
        vo_cov.block<3,3>(6,6).diagonal() = (delta_t*visual_orientation_sigma_).cwiseProduct(delta_t*visual_orientation_sigma_);
        vo_cov.block<3,3>(9,9).diagonal() = (delta_t*visual_omega_sigma_).cwiseProduct(delta_t*visual_omega_sigma_);
        vo_cov.block<3,3>(6,9).diagonal() = (delta_t*delta_t/2.0*visual_omega_sigma_).cwiseProduct(delta_t*delta_t/2.0*visual_omega_sigma_);
        vo_cov.block<3,3>(9,6).diagonal() = (delta_t*delta_t/2.0*visual_omega_sigma_).cwiseProduct(delta_t*delta_t/2.0*visual_omega_sigma_);
        vo_cov_rot = cov_rot*vo_cov*cov_rot.transpose();
    }

    meas_cov.block<2,2>(0,0) = vo_cov_rot.block<2,2>(0,0);
    meas_cov.block<2,2>(0,2) = vo_cov_rot.block<2,2>(0,3);
    meas_cov.block<2,6>(0,4) = vo_cov_rot.block<2,6>(0,6);
    meas_cov.block<2,2>(2,0) = vo_cov_rot.block<2,2>(2,0);
    meas_cov.block<2,2>(2,2) = vo_cov_rot.block<2,2>(3,3);
    meas_cov.block<2,6>(2,4) = vo_cov_rot.block<2,6>(2,6);
    meas_cov.block<3,3>(4,4) = vo_cov_rot.block<3,3>(6,6);
    meas_cov.block<3,3>(7,7) = vo_cov_rot.block<3,3>(9,9);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt>0)
    {
        ROS_DEBUG_STREAM("Performing visual predict with dt=" << dt);
        ROS_DEBUG_STREAM("Process noise:\n" << process_noise(dt));
        ukf_->predict(dt, process_noise(dt));
        last_update_ = msg->header.stamp;
    }

    ROS_DEBUG_STREAM("Visual correct:\n" << m);
    ROS_DEBUG_STREAM("Visual measurement covariance:\n" << meas_cov);
    ukf_->correct(m, meas_covs);
    last_visual_state_ = ukf_->state();
    last_visual_covariance_ = ukf_->covariance();
    last_visual_odom_ = *msg;
    //printState();
    sendState();
}

void
PoseUKFNode::parseWheelParameters(const ros::NodeHandle& privatenh)
{
    // retrieve default wheel parameters
    privatenh.param("wheel_forward_noise", wheel_forward_noise_, 0.002);
    privatenh.param("wheel_perpendicular_noise", wheel_perpendicular_noise_, 0.001);
    privatenh.param("wheel_velocity_noise", wheel_velocity_noise_, 0.001);
    privatenh.param("wheel_diameter", wheel_diameter_, 0.13);
    privatenh.param("large_rotation_jump", large_rotation_jump_, 1024.0);
    privatenh.param("large_steering_jump", large_steering_jump_, 1024.0);
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
        std::shared_ptr<struct Wheel> w(new struct Wheel(wheel_diameter_,
                    wheel_forward_noise_, wheel_perpendicular_noise_,
                    wheel_velocity_noise_,
                    large_rotation_jump_, large_steering_jump_,
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

void
PoseUKFNode::parseVisualMeasurementSigma(const ros::NodeHandle& privatenh)
{
    if(!privatenh.hasParam("visual_sigma"))
    {
        use_vo_covariance_ = true;
        return;
    }
    XmlRpc::XmlRpcValue visual_sigma;
    privatenh.getParam("visual_sigma", visual_sigma);
    ROS_ASSERT(visual_sigma.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(visual_sigma.hasMember(std::string("position")))
    {
        XmlRpc::XmlRpcValue list = visual_sigma[std::string("position")];
        listToVec(list, &sigma_position_);
    }
    if(visual_sigma.hasMember(std::string("velocity")))
    {
        XmlRpc::XmlRpcValue list = visual_sigma[std::string("velocity")];
        listToVec(list, &sigma_velocity_);
    }
    if(visual_sigma.hasMember(std::string("orientation")))
    {
        XmlRpc::XmlRpcValue list = visual_sigma[std::string("orientation")];
        listToVec(list, &sigma_orientation_);
    }
    if(visual_sigma.hasMember(std::string("omega")))
    {
        XmlRpc::XmlRpcValue list = visual_sigma[std::string("omega")];
        listToVec(list, &sigma_omega_);
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseUKF");
    PoseUKF::PoseUKFNode n;
    while(!n.initialize() && ros::isShuttingDown())
        ros::Duration(1.0).sleep();
    n.connect();
    ros::spin();
}
