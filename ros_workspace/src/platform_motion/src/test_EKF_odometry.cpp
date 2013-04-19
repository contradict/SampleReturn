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

    MatrixWrapper::Matrix A(3,3);
    A(1,1) = 1.0; A(1,2) = 0.0; A(1,3) = 0.0;
    A(2,1) = 0.0; A(2,2) = 1.0; A(2,3) = 0.0;
    A(3,1) = 0.0; A(3,2) = 0.0; A(3,3) = 1.0;

    std::vector<MatrixWrapper::Matrix> vA(1);
    vA[0] = A;

    // create gaussian
    MatrixWrapper::ColumnVector sysNoise_Mu(3);
    sysNoise_Mu = 0;

    double sigma_system_noise_vx = 0.05;
    double sigma_system_noise_vy = 0.05;
    double sigma_system_noise_omega = 0.10;

    MatrixWrapper::SymmetricMatrix sysNoise_Cov(3);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(1,1) = sigma_system_noise_vx;
    sysNoise_Cov(2,2) = sigma_system_noise_vy;
    sysNoise_Cov(3,3) = sigma_system_noise_omega;

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

    double sigma_prior_vel = 0.05;
    double sigma_prior_omega = 0.01;

    MatrixWrapper::ColumnVector prior_Mu(3);
    prior_Mu = 0;
    MatrixWrapper::SymmetricMatrix prior_Cov(3);
    prior_Cov = 0;
    prior_Cov(1,1) = sigma_prior_vel;
    prior_Cov(2,2) = sigma_prior_vel;
    prior_Cov(3,3) = sigma_prior_omega;
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
