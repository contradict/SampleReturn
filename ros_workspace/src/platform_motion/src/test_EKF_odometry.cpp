#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <filter/extendedkalmanfilter.h>
#include <pdf/gaussian.h>

#include "odometry/wheelpodmeasurementpdf.h"

#include <iostream>

int main(int argc, char** argv)
{
    BFL::LinearAnalyticConditionalGaussian *sys_pdf;
    BFL::LinearAnalyticSystemModelGaussianUncertainty *sys_model;
    BFL::AnalyticConditionalGaussian *meas_pdf;
    BFL::AnalyticMeasurementModelGaussianUncertainty *meas_model;
    BFL::Gaussian *prior;
    BFL::ExtendedKalmanFilter *filter;

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

    double sigma_system_noise_x = 0.01;
    double sigma_system_noise_y = 0.01;
    double sigma_system_noise_theta = 0.02;
    double sigma_system_noise_vx = 0.05;
    double sigma_system_noise_vy = 0.05;
    double sigma_system_noise_omega = 0.10;

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
    double sigma_meas_noise_v = 0.002;

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
    pod_positions(1,1) =  0.0;   pod_positions(2,1) =  0.65;
    pod_positions(1,2) =  0.0;   pod_positions(2,2) = -0.65;
    pod_positions(1,3) = -1.126; pod_positions(2,3) =  0.00;
    meas_pdf = new platform_motion::WheelPodMeasurementPdf(measurement_Uncertainty, pod_positions);
    // create the model
    meas_model = new BFL::AnalyticMeasurementModelGaussianUncertainty(meas_pdf);

    /****************************
     * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)

    double sigma_prior_pos = 1.00;
    double sigma_prior_theta = 0.10;
    double sigma_prior_vel = 0.05;
    double sigma_prior_omega = 0.01;

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

    for(int i=0;i<100;i++)
    {
        MatrixWrapper::ColumnVector measurement(3);
        measurement(1) = 0.1;
        measurement(2) = 0.1;
        measurement(3) = 0.1;

        MatrixWrapper::ColumnVector input(3);
        input(1) = 0;
        input(2) = 0;
        input(3) = 0;

        // UPDATE FILTER
        filter->Update(sys_model,meas_model,measurement,input);
        //filter->Update(sys_model);

        BFL::Pdf<MatrixWrapper::ColumnVector> * posteriorPDF = filter->PostGet();
        MatrixWrapper::ColumnVector posterior(posteriorPDF->ExpectedValueGet());

        std::cout << i << ": " << posterior << std::endl;
    }

}
