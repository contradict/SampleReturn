#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/gaussian.h>
#include <iostream>

#include "odometry/EKF_odometry.h"

#define MEASMODEL_NUMCONDARGUMENTS_THREEWHEEL 2
#define MEASMODEL_DIMENSION_THREEWHEEL        3

namespace platform_motion
{

std::ostream & operator<< (std::ostream &out, const odometry_measurements &m)
{
    out << "port: (" << m.port_pos.transpose() << ") (" << m.port_dir.transpose() << ") " << m.port_distance << " " << m.port_delta << " " << m.port_vel << " " << m.port_angle << std::endl;
    out << "stern: (" << m.stern_pos.transpose() << ") (" << m.stern_dir.transpose() << ") " << m.stern_distance << " " << m.stern_delta << " " << m.stern_vel << " " << m.stern_angle << std::endl;
    out << "starboard: (" << m.starboard_pos.transpose() << ") (" << m.starboard_dir.transpose() << ") " << m.starboard_distance << " " << m.starboard_delta << " " << m.starboard_vel << " " << m.starboard_angle << std::endl;
    return out;
}


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
     * Linear system model      *
     ***************************/

    double dt=0.050;
    MatrixWrapper::Matrix A(6,6);
    A(1,1) = 1.0; A(1,2) = 0.0; A(1,3) = 0.0; A(1,4) = 1.0*dt; A(1,5) = 0.0; A(1,6) = 0.0;
    A(2,1) = 0.0; A(2,2) = 1.0; A(2,3) = 0.0; A(2,4) = 0.0; A(2,5) = 1.0*dt; A(2,6) = 0.0;
    A(3,1) = 0.0; A(3,2) = 0.0; A(3,3) = 1.0; A(3,4) = 0.0; A(3,5) = 0.0; A(3,6) = 1.0*dt;
    A(4,1) = 0.0; A(4,2) = 0.0; A(4,3) = 0.0; A(4,4) = 1.0; A(4,5) = 0.0; A(4,6) = 0.0;
    A(5,1) = 0.0; A(5,2) = 0.0; A(5,3) = 0.0; A(5,4) = 0.0; A(5,5) = 1.0; A(5,6) = 0.0;
    A(6,1) = 0.0; A(6,2) = 0.0; A(6,3) = 0.0; A(6,4) = 0.0; A(6,5) = 0.0; A(6,6) = 1.0;

    std::vector<MatrixWrapper::Matrix> vA(1);
    vA[0] = A;

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
    sys_pdf = new BFL::LinearAnalyticConditionalGaussian(vA, system_Uncertainty);
    // create the system model
    sys_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(sys_pdf);

    /*********************************
     * Initialise measurement model *
     ********************************/

    MatrixWrapper::ColumnVector measNoise_Mu(3);
    measNoise_Mu = 0;
    MatrixWrapper::SymmetricMatrix measNoise_Cov(3);
    measNoise_Cov = 0;
    measNoise_Cov(1,1) = sigma_meas_noise_v;
    measNoise_Cov(2,2) = sigma_meas_noise_v;
    measNoise_Cov(3,3) = sigma_meas_noise_v;
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
    std::cerr << data;
    MatrixWrapper::ColumnVector measurement(3);
    measurement(1) = data.port_delta/data.interval;
    measurement(2) = data.starboard_delta/data.interval;
    measurement(3) = data.stern_delta/data.interval;

    MatrixWrapper::ColumnVector input(3);
    input(1) = data.port_angle;
    input(2) = data.starboard_angle;
    input(3) = data.stern_angle;

    // UPDATE FILTER
    filter->Update(sys_model,meas_model,measurement,input);

    BFL::Pdf<MatrixWrapper::ColumnVector> * posteriorPDF = filter->PostGet();
    MatrixWrapper::ColumnVector posterior(posteriorPDF->ExpectedValueGet());

    odom_position[0] = posterior(1);
    odom_position[1] = posterior(2);
    odom_orientation = posterior(3);
    while(odom_orientation > 2*M_PI) odom_orientation -= 2*M_PI;
    while(odom_orientation < 0) odom_orientation += 2*M_PI;
    odom_velocity[0] = posterior(4);
    odom_velocity[1] = posterior(5);
    odom_omega = posterior(6);

    resetReferencePose(data, stamp);

}

}
