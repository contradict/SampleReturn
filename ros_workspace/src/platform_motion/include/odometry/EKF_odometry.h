
#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <filter/extendedkalmanfilter.h>

#include "odometry/odometry_node.h"
#include "odometry/wheelpodmeasurementpdf.h"
#include "odometry/odometrymotionpdf.h"

namespace platform_motion
{

class EKFOdometryNode: public OdometryNode
{
    public:
        EKFOdometryNode(void);
        virtual void init(void);

    private:
        void initializeModels(void);

        double sigma_system_noise_x, sigma_system_noise_y, sigma_system_noise_theta,
               sigma_system_noise_vx, sigma_system_noise_vy, sigma_system_noise_omega;
        double sigma_meas_noise_v;
        double sigma_prior_pos, sigma_prior_theta,
               sigma_prior_vel, sigma_prior_omega;

        BFL::AnalyticConditionalGaussian *sys_pdf;
        BFL::AnalyticSystemModelGaussianUncertainty *sys_model;
        BFL::AnalyticConditionalGaussian *meas_pdf;
        BFL::AnalyticMeasurementModelGaussianUncertainty *meas_model;
        BFL::Gaussian *prior;
        BFL::ExtendedKalmanFilter *filter;

        virtual void computeOdometry(struct odometry_measurements &data, const ros::Time &stamp);

};

}
