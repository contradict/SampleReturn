#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/saliency_detector_paramsConfig.h>

#include "BMS.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class SaliencyDetector
{
  public:
    SaliencyDetector();
    ~SaliencyDetector();
    void messageCallback(const sensor_msgs::ImageConstPtr& msg);
    void configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level);
    void publishMessage(ros::Publisher *pub_message);

  private:
    cv::Mat debug_bms_img_;
    BMS bms_;
    int bms_sample_step_;
    double bms_blur_std_;
};
