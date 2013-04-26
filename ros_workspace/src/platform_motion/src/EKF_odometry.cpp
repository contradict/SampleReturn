#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/gaussian.h>
#include <iostream>

#include "odometry/EKF_odometry.h"

#define MEASMODEL_NUMCONDARGUMENTS_THREEWHEEL 2
#define MEASMODEL_DIMENSION_THREEWHEEL        3

namespace platform_motion
{

EKFOdometryNode::EKFOdometryNode(void):OdometryNode()
{
    ros::NodeHandle param_nh("~");

    param_nh.param("sigma_system_noise_x", sigma_system_noise_x, 0.01);
    param_nh.param("sigma_system_noise_y", sigma_system_noise_y, 0.01);
    param_nh.param("sigma_system_noise_theta", sigma_system_noise_theta, 0.02);
    param_nh.param("sigma_system_noise_vx", sigma_system_noise_vx, 0.05);
    param_nh.param("sigma_system_noise_vy", sigma_system_noise_vy, 0.05);
    param_nh.param("sigma_system_noise_omega", sigma_system_noise_omega, 0.10);

    param_nh.param("sigma_meas_noise_v", sigma_meas_noise_v, 0.002);

    param_nh.param("sigma_prior_pos", sigma_prior_pos, 1.00);
    param_nh.param("sigma_prior_theta", sigma_prior_theta, 0.10);
    param_nh.param("sigma_prior_vel", sigma_prior_vel, 0.05);
    param_nh.param("sigma_prior_omega", sigma_prior_omega, 0.01);
}

void EKFOdometryNode::init(void)
{
    OdometryNode::init();

    initializeModels();
}

void EKFOdometryNode::initializeModels(void)
{
    /****************************
     * NonLinear system model      *
     ***************************/

    // create gaussian
    MatrixWrapper::ColumnVector sysNoise_Mu(6);
    sysNoise_Mu = 0;

    MatrixWrapper::SymmetricMatrix sysNoise_Cov(6);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(1,1) = sigma_system_noise_x;
    sysNoise_Cov(2,2) = sigma_system_noise_y;
    sysNoise_Cov(3,3) = sigma_system_noise_theta;
    sysNoise_Cov(4,4) = sigma_system_noise_vx;
    sysNoise_Cov(5,5) = sigma_system_noise_vy;
    sysNoise_Cov(6,6) = sigma_system_noise_omega;

    BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf = new OdometryMotionPdf(system_Uncertainty);
    // create the system model
    sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf);

    /*********************************
     * Initialise measurement model *
     ********************************/

    MatrixWrapper::ColumnVector measNoise_Mu(6);
    measNoise_Mu = 0;
    MatrixWrapper::SymmetricMatrix measNoise_Cov(6);
    measNoise_Cov = 0;
    measNoise_Cov(1,1) = sigma_meas_noise_v;
    measNoise_Cov(2,2) = sigma_meas_noise_v;
    measNoise_Cov(3,3) = sigma_meas_noise_v;
    measNoise_Cov(4,4) = sigma_meas_noise_v;
    measNoise_Cov(5,5) = sigma_meas_noise_v;
    measNoise_Cov(6,6) = sigma_meas_noise_v;

    BFL::Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);
    MatrixWrapper::Matrix pod_positions(3,3);
    pod_positions = 0;
    pod_positions(1,1) = data.port_pos[0];      pod_positions(2,1) = data.port_pos[1];
    pod_positions(1,2) = data.starboard_pos[0]; pod_positions(2,2) = data.starboard_pos[1];
    pod_positions(1,3) = data.stern_pos[0];     pod_positions(2,3) = data.stern_pos[1];
    meas_pdf = new WheelPodMeasurementPdf(measurement_Uncertainty, pod_positions);
    // create the model
    meas_model = new BFL::AnalyticMeasurementModelGaussianUncertainty(meas_pdf);

    /****************************
     * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    MatrixWrapper::ColumnVector prior_Mu(6);
    prior_Mu = 0;
    MatrixWrapper::SymmetricMatrix prior_Cov(6);
    prior_Cov = 0;
    prior_Cov(1,1) = sigma_prior_pos;
    prior_Cov(2,2) = sigma_prior_pos;
    prior_Cov(3,3) = sigma_prior_theta;
    prior_Cov(4,4) = sigma_prior_vel;
    prior_Cov(5,5) = sigma_prior_vel;
    prior_Cov(6,6) = sigma_prior_omega;
    prior = new BFL::Gaussian(prior_Mu,prior_Cov);

    /******************************
     * Construction of the Filter *
     ******************************/
    filter = new BFL::ExtendedKalmanFilter(prior);
}

void EKFOdometryNode::computeOdometry(struct odometry_measurements &data, const ros::Time &stamp)
{
    //std::cerr << data;
    MatrixWrapper::ColumnVector measurement(6);
    measurement(1) = data.port_delta;
    measurement(2) = 0.0;
    measurement(3) = data.starboard_delta;
    measurement(4) = 0.0;
    measurement(5) = data.stern_delta;
    measurement(6) = 0.0;

    MatrixWrapper::ColumnVector input(3);
    input(1) = data.port_angle;
    input(2) = data.starboard_angle;
    input(3) = data.stern_angle;

    // UPDATE FILTER
    filter->Update(sys_model,meas_model,measurement,input);

    BFL::Pdf<MatrixWrapper::ColumnVector> * posteriorPDF = filter->PostGet();
    MatrixWrapper::ColumnVector posterior(posteriorPDF->ExpectedValueGet());
    MatrixWrapper::SymmetricMatrix posteriorCov(posteriorPDF->CovarianceGet());

    // emit in odometry frame
    MatrixWrapper::Matrix R(2,2);
    R(1,1) = cos(odom_orientation); R(1,2) = -sin(odom_orientation);
    R(2,1) = sin(odom_orientation); R(2,2) =  cos(odom_orientation);
    MatrixWrapper::ColumnVector OdometryFrameVelocity = R*posterior.sub(4,5)/data.interval;
    odom_position[0] = posterior(1);
    odom_position[1] = posterior(2);
    odom_orientation = posterior(3);
    while(odom_orientation > 2*M_PI) odom_orientation -= 2*M_PI;
    while(odom_orientation < 0) odom_orientation += 2*M_PI;
    odom_velocity[0] = OdometryFrameVelocity(1);
    odom_velocity[1] = OdometryFrameVelocity(2);
    odom_omega = posterior(6)/data.interval;
    // update covariance
    // odom_pose_* is x,y,yaw,roll,pitch,yaw
    // odom_twist_* is vx,vy,vyaw,omega_roll,omega_pitch,omgea_yaw
    // posterior is x,y,yaw,vx,vy,omega_yaw
    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
        {
            // copy x,y
            odom_pose_covariance[i*6+j] = posteriorCov(i+1, j+1);
            // copy vx,vy
            odom_twist_covariance[i*6+j] = posteriorCov(i+4, j+4);
        }
    for(int i=0;i<2;i++)
    {
        // copy theta-x,y row
        odom_pose_covariance[3*6+i] = posteriorCov(3,i+1);
        odom_twist_covariance[3*6+i] = posteriorCov(6, i+4);
        // copy theta-x,y column
        odom_pose_covariance[i*6+3] = posteriorCov(i+1,3);
        odom_twist_covariance[i*6+3] = posteriorCov(i+4, 6);
    }
    // copy theta-theta value
    odom_pose_covariance[3*6+3] = posteriorCov(3, 3);
    odom_twist_covariance[3*6+3] = posteriorCov(6, 6);

    resetReferencePose(data, stamp);

}

}
