#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <pose_ukf/pose_ukf.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <functional>


namespace PoseUKF {

bool lookupTransform(const tf::TransformListener& listener,
        std::string base_name,
        std::string link_name,
        std::string err,
        tf::StampedTransform* transform)
{
    std::string errmsg;
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

    std::vector<std::shared_ptr<struct Wheel> > wheels_;
    std::map<std::string, std::shared_ptr<struct Wheel> > steering_values_;
    std::map<std::string, std::shared_ptr<struct Wheel> > rotation_values_;

    void jointStateCallback(sensor_msgs::JointStateConstPtr msg);
    void imuCallback(sensor_msgs::ImuConstPtr msg);

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    ros::Subscriber imu_subscription_;
    ros::Subscriber joint_subscription_;

    PoseUKF *ukf_;
    ros::Time last_update_;
    ros::Time last_joint_state_;
    ros::Time last_imu_;

    struct PoseState wheel_last_pose_;

    void parseWheelParameters(const ros::NodeHandle& privatenh);
    void parseProcessSigma(const ros::NodeHandle& privatenh);

    public:
        PoseUKFNode();
        ~PoseUKFNode();
        bool initialize(void);
        void connect(void);
        void printState(void);
};

PoseUKFNode::PoseUKFNode() :
    ukf_(NULL)
{
    ros::NodeHandle privatenh("~");
    ros::NodeHandle nh("~");

    privatenh.param("base_name", base_name_, std::string("base_link"));

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
}

void
PoseUKFNode::connect(void)
{
    ros::NodeHandle nh;
    last_update_ = ros::Time::now();
    last_joint_state_ = last_update_;
    wheel_last_pose_ = ukf_->state();
    last_imu_ = last_update_;
    imu_subscription_ = nh.subscribe("imu", 1, &PoseUKFNode::imuCallback, this);
    joint_subscription_ = nh.subscribe("joint_state", 1, &PoseUKFNode::jointStateCallback, this);
    ROS_INFO("Subscribers created");
}

PoseUKFNode::~PoseUKFNode()
{
    delete ukf_;
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
PoseUKFNode::imuCallback(sensor_msgs::ImuConstPtr msg)
{
    IMUOrientationMeasurement m;
    tf::vectorMsgToEigen(msg->linear_acceleration,
                         m.acceleration);
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
    meas_cov.block<3, 3>(3, 3) = omega_cov;
    ROS_INFO_STREAM("imu omega cov:\n" << omega_cov);
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(meas_cov);

    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt<0)
        return;
    ROS_INFO_STREAM("Performing IMU update with dt=" << dt << ", delta_t=" << m.delta_t);
    ROS_INFO_STREAM(m);
    ukf_->predict(dt, meas_covs, false);
    last_update_ = msg->header.stamp;
    ukf_->correct(m);
    printState();
}

void
PoseUKFNode::printState(void)
{
    ROS_INFO_STREAM("State:\n" << ukf_->state());
    ROS_INFO_STREAM("position cov:\n" << (ukf_->covariance().block<3,3>(0,0)));
    ROS_INFO_STREAM("velocity cov:\n" << (ukf_->covariance().block<3,3>(3,3)));
    ROS_INFO_STREAM("orientation cov:\n" << (ukf_->covariance().block<3,3>(6,6)));
    ROS_INFO_STREAM("omega cov:\n" << (ukf_->covariance().block<3,3>(9,9)));
    ROS_INFO_STREAM("gyro bias cov:\n" << (ukf_->covariance().block<3,3>(12,12)));
    ROS_INFO_STREAM("accel bias cov:\n" << (ukf_->covariance().block<3,3>(15,15)));
}

void
PoseUKFNode::jointStateCallback(sensor_msgs::JointStateConstPtr msg)
{
    if(wheels_.size() == 0)
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

    struct WheelOdometryMeasurement m;

    m.delta_t = (msg->header.stamp - last_joint_state_).toSec();
    last_joint_state_ = msg->header.stamp;
    m.base_state = wheel_last_pose_;

    m.wheel_positions.resize(wheels_.size(), 3);
    m.wheel_motions.resize(wheels_.size(), 2);
    m.wheel_velocities.resize(wheels_.size());
    Eigen::MatrixXd mcov(m.ndim(), m.ndim());
    mcov.setZero();
    bool good_delta=true;
    for(size_t i=0; i<wheels_.size(); i++)
    {
        double dphi, dtheta;
        good_delta &= wheels_[i]->delta(&dphi, &dtheta);
        double phi = wheels_[i]->steering_angle;
        Eigen::Vector2d motion(cos(phi), sin(phi));
        motion *= dtheta*wheels_[i]->diameter;
        m.wheel_motions.row(i) = motion;
        m.wheel_positions.row(i) = wheels_[i]->position;
        m.wheel_velocities(i) = wheels_[i]->rotation_velocity*wheels_[i]->diameter;
        mcov.block<2,2>(3*i, 3*i) << cos(phi)*wheels_[i]->forward_noise, -sin(phi)*wheels_[i]->perpendicular_noise,
                    sin(phi)*wheels_[i]->forward_noise,  cos(phi)*wheels_[i]->perpendicular_noise;
        mcov(3*i+2, 3*i+2) = wheels_[i]->velocity_noise;
        ROS_INFO_STREAM("wheel measurement(" << i << "):\n" << m.wheel_motions.row(i));
        ROS_INFO_STREAM("wheel measurement covariance(" << i << "):\n" << (mcov.block<2,2>(2*i, 2*i)));
    }
    if(!good_delta)
    {
        ROS_ERROR("Large angle jump, skipping odometry update");
        return;
    }
    std::vector<Eigen::MatrixXd> meas_covs;
    meas_covs.push_back(mcov);


    double dt = (msg->header.stamp - last_update_).toSec();
    if(dt<0)
        return;
    ROS_INFO_STREAM("Performing joint state update with dt=" << dt);
    ROS_INFO_STREAM(m);
    ukf_->predict(dt, meas_covs, false);
    last_update_ = msg->header.stamp;
    ukf_->correct(m);
    wheel_last_pose_ = ukf_->state();
    printState();
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

void listToVec(XmlRpc::XmlRpcValue& list, Eigen::Vector3d *vec)
{
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i<std::min(3, list.size()); i++)
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
        listToVec(list, &ukf_->sigma_position);
    }
    if(process_sigma.hasMember(std::string("velocity")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("velocity")];
        listToVec(list, &ukf_->sigma_velocity);
    }
    if(process_sigma.hasMember(std::string("orientation")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("orientation")];
        listToVec(list, &ukf_->sigma_orientation);
    }
    if(process_sigma.hasMember(std::string("omega")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("omega")];
        listToVec(list, &ukf_->sigma_omega);
    }
    if(process_sigma.hasMember(std::string("gyro_bias")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("gyro_bias")];
        listToVec(list, &ukf_->sigma_gyro_bias);
    }
    if(process_sigma.hasMember(std::string("accel_bias")))
    {
        XmlRpc::XmlRpcValue list = process_sigma[std::string("accel_bias")];
        listToVec(list, &ukf_->sigma_accel_bias);
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseUKF");
    PoseUKF::PoseUKFNode n;
    while(!n.initialize())
        ros::Duration(1.0).sleep();
    n.connect();
    ros::spin();
}