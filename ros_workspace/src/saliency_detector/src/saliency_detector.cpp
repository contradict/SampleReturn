//#include "saliency_detector_core.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/saliency_detector_paramsConfig.h>

#include "BMS.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class SaliencyDetectorNode
{
  ros::NodeHandle nh;
  ros::Subscriber sub_img;
  ros::Publisher pub_bms_img;
  string img_topic;
  string bms_debug_topic;

  cv::Mat debug_bms_img_;
  BMS bms_;
  int bms_sample_step_;
  double bms_blur_std_;

  public:
  SaliencyDetectorNode() {

    dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&SaliencyDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("img_topic", img_topic, string("/cam_img"));
    private_node_handle_.param("bms_debug_topic", bms_debug_topic, string("/bms_img"));

    sub_img =
      nh.subscribe(img_topic.c_str(), 3, &SaliencyDetectorNode::messageCallback, this);

    pub_bms_img =
      nh.advertise<sensor_msgs::Image>(bms_debug_topic.c_str(), 3);
  }

  void messageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("messageCallback");
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "");
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat small;
    cv::resize(cv_ptr->image,small,cv::Size(600.0,cv_ptr->image.rows*(600.0/cv_ptr->image.cols)),0.0,0.0,cv::INTER_AREA);

    bms_.computeSaliency(small, bms_sample_step_);
    debug_bms_img_ = bms_.getSaliencyMap().clone();

    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"mono8",debug_bms_img_).toImageMsg();
    pub_bms_img.publish(debug_img_msg);

    ROS_INFO("messageCallback ended");
  }

  /* Dynamic reconfigure callback */
  void configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level)
  {
    ROS_INFO("configCallback");
    // Construct BMS
    bms_ = BMS(config.bms_dilation_width_1, config.bms_opening_width,
        config.bms_normalize, config.bms_handle_border);

    bms_sample_step_ = config.bms_sample_step;
    bms_blur_std_ = config.bms_blur_std;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detector");
  SaliencyDetectorNode sd;
  ros::spin();
}
