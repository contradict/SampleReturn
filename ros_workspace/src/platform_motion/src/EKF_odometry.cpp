#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/gaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <iostream>

#include "odometry/EKF_odometry.h"
#include "odometry/odometrywithcurvature.h"

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
    param_nh.param("sigma_system_noise_curvature", sigma_system_noise_curvature, 0.001);

    param_nh.param("sigma_wheel_meas_noise_v", sigma_wheel_meas_noise_v, 0.002);

    param_nh.param("sigma_prior_pos", sigma_prior_pos, 1.00);
    param_nh.param("sigma_prior_theta", sigma_prior_theta, 0.10);
    param_nh.param("sigma_prior_vel", sigma_prior_vel, 0.05);
    param_nh.param("sigma_prior_omega", sigma_prior_omega, 0.01);
    param_nh.param("sigma_prior_curvature", sigma_prior_curvature, 0.001);

    param_nh.param("max_yaw_correction_rate", max_yaw_correction_rate, 0.05);

    yaw_sub = param_nh.subscribe("sun_yaw", 2, &EKFOdometryNode::updateYaw, this);
}

void EKFOdometryNode::init(void)
{
    OdometryNode::init();

    initializeModels();
}

void EKFOdometryNode::initializeModels(void)
{
    /****************************
     * Nonlinear system model      *
     ***************************/


    // create gaussian
    MatrixWrapper::ColumnVector sysNoise_Mu(5);
    sysNoise_Mu = 0;

    MatrixWrapper::SymmetricMatrix sysNoise_Cov(5);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(1,1) = sigma_system_noise_vx;
    sysNoise_Cov(2,2) = sigma_system_noise_vy;
    sysNoise_Cov(3,3) = sigma_system_noise_omega;
    sysNoise_Cov(4,4) = sigma_system_noise_theta;
    sysNoise_Cov(5,5) = sigma_system_noise_curvature;

    BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf = new platform_odometry::OdometryWithCurvature(system_Uncertainty);
    // create the system model
    sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf);

    /****************************************
    * Initialise odometry measurement model *
    ****************************************/

    MatrixWrapper::ColumnVector wheelmeasNoise_Mu(6);
    wheelmeasNoise_Mu = 0;
    MatrixWrapper::SymmetricMatrix wheelmeasNoise_Cov(6);
    wheelmeasNoise_Cov = 0;
    wheelmeasNoise_Cov(1,1) = sigma_wheel_meas_noise_v;
    wheelmeasNoise_Cov(2,2) = sigma_wheel_meas_noise_v;
    wheelmeasNoise_Cov(3,3) = sigma_wheel_meas_noise_v;
    wheelmeasNoise_Cov(4,4) = sigma_wheel_meas_noise_v;
    wheelmeasNoise_Cov(5,5) = sigma_wheel_meas_noise_v;
    wheelmeasNoise_Cov(6,6) = sigma_wheel_meas_noise_v;

    BFL::Gaussian wheel_measurement_Uncertainty(wheelmeasNoise_Mu, wheelmeasNoise_Cov);
    MatrixWrapper::Matrix pod_positions(3,3);
    pod_positions = 0;
    pod_positions(1,1) = data.port_pos[0];      pod_positions(2,1) = data.port_pos[1];
    pod_positions(1,2) = data.starboard_pos[0]; pod_positions(2,2) = data.starboard_pos[1];
    pod_positions(1,3) = data.stern_pos[0];     pod_positions(2,3) = data.stern_pos[1];
    wheel_meas_pdf = new WheelPodMeasurementPdf(wheel_measurement_Uncertainty, pod_positions);
    // create the model
    wheel_meas_model = new BFL::AnalyticMeasurementModelGaussianUncertainty(wheel_meas_pdf);

    /***********************************
    * Initialize sun measurement model 
    *  set uncertianty to N(0,1) for now, This will be updated by the
    *  covariance data from the message
    ***********************************/
    MatrixWrapper::ColumnVector sunNoiseMu(1);
    sunNoiseMu = 0.0;
    MatrixWrapper::SymmetricMatrix sunNoiseCov(1);
    sunNoiseCov = 1.0;
    BFL::Gaussian sunUncertainty(sunNoiseMu, sunNoiseCov);
    MatrixWrapper::Matrix Hsun(1,5);
    Hsun = 0.0;
    Hsun(1,4) = 1.0;
    std::vector<MatrixWrapper::Matrix> Hmats;
    Hmats.push_back(Hsun);
    sun_meas_pdf = new BFL::LinearAnalyticConditionalGaussian(Hmats, sunUncertainty);
    sun_meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(sun_meas_pdf);

    /****************************
     * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    MatrixWrapper::ColumnVector prior_Mu(5);
    prior_Mu = 0;
    MatrixWrapper::SymmetricMatrix prior_Cov(5);
    prior_Cov = 0;
    prior_Cov(1,1) = sigma_prior_vel;
    prior_Cov(2,2) = sigma_prior_vel;
    prior_Cov(3,3) = sigma_prior_omega;
    prior_Cov(4,4) = sigma_prior_theta;
    prior_Cov(5,5) = sigma_prior_curvature;
    prior = new BFL::Gaussian(prior_Mu,prior_Cov);

    /******************************
     * Construction of the Filter *
     ******************************/
    filter = new BFL::ExtendedKalmanFilter(prior);
}

void EKFOdometryNode::computeOdometry(struct odometry_measurements &data, const ros::Time &stamp)
{
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
    filter->Update(sys_model,wheel_meas_model,measurement,input);

    BFL::Pdf<MatrixWrapper::ColumnVector> * posteriorPDF = filter->PostGet();
    MatrixWrapper::ColumnVector posterior(posteriorPDF->ExpectedValueGet());
    MatrixWrapper::SymmetricMatrix posteriorCov(posteriorPDF->CovarianceGet());

    // integrate in odometry frame
    MatrixWrapper::Matrix R(2,2);
    R(1,1) = cos(posterior(4)); R(1,2) = -sin(posterior(4));
    R(2,1) = sin(posterior(4)); R(2,2) =  cos(posterior(4));
    MatrixWrapper::ColumnVector OdometryFrameVelocity = R*posterior.sub(1,2);
    odom_position[0] += OdometryFrameVelocity(1);
    odom_position[1] += OdometryFrameVelocity(2);
    odom_velocity[0] = posterior(1)/data.interval;
    odom_velocity[1] = posterior(2)/data.interval;
    odom_omega = posterior(3)/data.interval;
    double tsq = data.interval*data.interval;
    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
        {
            // copy vx,vy
            odom_twist_covariance[i*6+j] = posteriorCov(i+1, j+1)/tsq;
        }
    for(int i=0;i<2;i++)
    {
        // copy omega-vx,vy row
        odom_twist_covariance[3*6+i] = posteriorCov(3, i+1)/tsq;
        // copy omega-vx,vy column
        odom_twist_covariance[i*6+3] = posteriorCov(i+1, 3)/tsq;
    }
    // copy omega-omega value
    odom_twist_covariance[3*6+3] = posteriorCov(3, 3)/tsq;

    resetReferencePose(data, stamp);

}

void EKFOdometryNode::updateYaw(const geometry_msgs::PoseWithCovarianceStampedConstPtr& yawmsg)
{
    MatrixWrapper::ColumnVector x=filter->PostGet()->ExpectedValueGet();
    if(fabs(x(3))>max_yaw_correction_rate)
        return;
    MatrixWrapper::ColumnVector measurement(1);
    measurement(1) = tf::getYaw(yawmsg->pose.pose.orientation);

    MatrixWrapper::SymmetricMatrix sunCov(1);
    sunCov(1,1) = yawmsg->pose.covariance[3*6+3];
    sun_meas_pdf->AdditiveNoiseSigmaSet(sunCov);
    filter->Update(sys_model,sun_meas_model,measurement);
}

}
