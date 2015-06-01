
#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <filter/extendedkalmanfilter.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "odometry/odometry_node.h"
#include "odometry/wheelpodmeasurementpdf.h"

namespace platform_motion
{

class EKFOdometryNode: public OdometryNode
{
    public:
        EKFOdometryNode(void);
        virtual void init(void);

    private:
        void initializeModels(void);
        void updateYaw(const geometry_msgs::PoseWithCovarianceStampedConstPtr& yawmsg);

        double sigma_system_noise_x, sigma_system_noise_y, sigma_system_noise_theta,
               sigma_system_noise_vx, sigma_system_noise_vy, sigma_system_noise_omega,
               sigma_system_noise_curvature;
        double sigma_wheel_meas_noise_v;
        double sigma_prior_pos, sigma_prior_theta,
               sigma_prior_vel, sigma_prior_omega,
               sigma_prior_curvature;
        double max_yaw_correction_rate;

        ros::Subscriber yaw_sub;

        BFL::AnalyticConditionalGaussianAdditiveNoise *sys_pdf;
        BFL::AnalyticSystemModelGaussianUncertainty *sys_model;
        BFL::AnalyticConditionalGaussian *wheel_meas_pdf;
        BFL::AnalyticMeasurementModelGaussianUncertainty *wheel_meas_model;
        BFL::LinearAnalyticConditionalGaussian *sun_meas_pdf;
        BFL::LinearAnalyticMeasurementModelGaussianUncertainty *sun_meas_model;
        BFL::Gaussian *prior;
        BFL::ExtendedKalmanFilter *filter;

        virtual void computeOdometry(struct odometry_measurements &data, const ros::Time &stamp);

};

}
