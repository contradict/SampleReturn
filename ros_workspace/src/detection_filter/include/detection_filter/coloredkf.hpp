#include <memory>
#include <samplereturn/colormodel.h>
#include <samplereturn_msgs/NamedPoint.h>

namespace detection_filter
{

class ColoredKF : public cv::KalmanFilter
{
  public:
    samplereturn::HueHistogram huemodel;
    int16_t filter_id;
    std::string frame_id;
    float certainty;
    template <typename configT>
    ColoredKF (configT config, samplereturn_msgs::NamedPoint msg, int16_t id);
};


template <typename configT>
ColoredKF::ColoredKF (configT config, samplereturn_msgs::NamedPoint msg, int16_t id) :
    cv::KalmanFilter(6,3),
    huemodel(msg.model.hue),
    filter_id(id),
    frame_id(msg.sensor_frame),
    certainty(config.PO_init)
{
    cv::Mat state(6, 1, CV_32F); /* x, y, z, vx, vy, vz */
    cv::Mat processNoise(6, 1, CV_32F);

    transitionMatrix = (cv::Mat_<float>(6,6) << 1, 0, 0, config.period, 0, 0,
                                                    0, 1, 0, 0, config.period, 0,
                                                    0, 0, 1, 0, 0, config.period,
                                                    0, 0, 0, 1, 0, 0,
                                                    0, 0, 0, 0, 1, 0,
                                                    0, 0, 0, 0, 0, 1);
    cv::setIdentity(measurementMatrix);
    cv::setIdentity(processNoiseCov, cv::Scalar(config.process_noise_cov));
    cv::setIdentity(measurementNoiseCov, cv::Scalar(config.measurement_noise_cov));
    cv::setIdentity(errorCovPost, cv::Scalar(config.error_cov_post));

    statePost.at<float>(0) = msg.point.x;
    statePost.at<float>(1) = msg.point.y;
    statePost.at<float>(2) = msg.point.z;
    statePost.at<float>(3) = 0;
    statePost.at<float>(4) = 0;
    statePost.at<float>(5) = 0;
}

}
