#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <samplereturn_msgs/PatchArray.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/saliency_detector_paramsConfig.h>

#include "BMS.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class SaliencyDetectorNode
{
  image_transport::ImageTransport *it;
  image_transport::CameraSubscriber sub_img;
  ros::Publisher pub_bms_img;
  ros::Publisher pub_patch_array;
  std::string img_topic;
  std::string bms_img_topic;
  std::string patch_array_topic;

  boost::mutex saliency_mutex_;

  cv::Mat debug_bms_img_;
  BMS bms_;
  cv::SimpleBlobDetector::Params blob_params_;
  cv::Ptr<cv::SimpleBlobDetector> blob_;
  bool blobDetect_on_;
  int bms_sample_step_;
  double bms_blur_std_;
  int bms_thresh_;
  bool bms_thresh_on_;
  double bms_top_trim_;
  double bms_img_width_;

  double scale_;

  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;

  public:
  SaliencyDetectorNode() {
    ros::NodeHandle nh;

    it = new image_transport::ImageTransport(nh);

    dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&SaliencyDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("img_topic", img_topic, std::string("/cameras/search/image"));

    private_node_handle_.param("bms_img_topic", bms_img_topic, std::string("bms_img"));
    private_node_handle_.param("patch_array_topic", patch_array_topic, std::string("patch_array"));

    sub_img =
      it->subscribeCamera(img_topic, 1, &SaliencyDetectorNode::messageCallback, this);

    pub_bms_img =
      nh.advertise<sensor_msgs::Image>(bms_img_topic.c_str(), 3);

    pub_patch_array =
      nh.advertise<samplereturn_msgs::PatchArray>(patch_array_topic.c_str(), 3);

    blob_params_.blobColor = 255;
    blob_params_.minArea = 15;
    blob_params_.maxArea = 600;
    blob_params_.filterByColor = true;
    blob_params_.filterByArea = true;
    blob_params_.filterByCircularity = false;
    blob_params_.filterByConvexity = false;
    blob_params_.filterByInertia = false;
    blob_params_.minDistBetweenBlobs = 50.;
    blob_ = cv::SimpleBlobDetector::create(blob_params_);
  }

  void messageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    saliency_mutex_.lock();
    ROS_DEBUG("messageCallback");
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "");
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Resize the image for BMS saliency computation
    ROS_DEBUG("Begin resize");
    cv::Mat small;
    scale_ = cv_ptr->image.cols/bms_img_width_;
    cv::resize(cv_ptr->image.rowRange(bms_top_trim_,cv_ptr->image.rows),
        small,cv::Size(bms_img_width_,(cv_ptr->image.rows-bms_top_trim_)*(bms_img_width_/cv_ptr->image.cols)),
        0.0,0.0,cv::INTER_NEAREST);
    ROS_DEBUG("End resize");

    // Compute saliency map
    ROS_DEBUG("Begin BMS Comp");
    bms_.computeSaliency(small, bms_sample_step_);
    debug_bms_img_ = bms_.getSaliencyMap().clone();
    ROS_DEBUG("End BMS Comp");

    // Threshold grayscale saliency into binary
    if (bms_thresh_on_) {
      ROS_DEBUG("Thresholding");
      cv::threshold(debug_bms_img_, debug_bms_img_, bms_thresh_, 255, cv::THRESH_BINARY);
    }

    // Allocate output image and detected region structures
    cv::Mat debug_bms_img_color;
    std::vector<cv::KeyPoint> kp;

    // Detect blobs matching criteria
    ROS_DEBUG("Begin Blob Comp");
    if (blobDetect_on_) {
      cv::Mat blob_copy = debug_bms_img_.clone();
      blob_->detect(blob_copy, kp);
      ROS_DEBUG("Keypoints Detected: %lu", kp.size());
    }
    ROS_DEBUG("End Blob Comp");

    // Scale back up
    ROS_DEBUG("Begin resize");
    cv::resize(debug_bms_img_, debug_bms_img_, cv::Size(cv_ptr->image.cols,cv_ptr->image.rows),
        0.0, 0.0, cv::INTER_NEAREST);
    cv::cvtColor(debug_bms_img_, debug_bms_img_color, CV_GRAY2RGB);
    ROS_DEBUG("End resize");

    // Allocate images and mask for patch publishing
    cv::Mat sub_img;
    cv::Mat sub_mask;
    samplereturn_msgs::PatchArray pa_msg;

    // Scale keypoint params back up from smaller BMS image
    ROS_DEBUG("Begin publish loop");
    for (size_t i = 0; i < kp.size(); i++) {
      int x = kp[i].pt.x * scale_;
      int y = kp[i].pt.y * scale_;
      // Pad a bit to avoid clipping
      int size = 1.2*kp[i].size * scale_;
      int top_left_x = max(x-size,0);
      int top_left_y = max(y-size,0);
      int bot_right_x = min(x+size,cv_ptr->image.cols - 1);
      int bot_right_y = min(y+size,cv_ptr->image.rows - 1);
      int width = bot_right_x - top_left_x;
      int height = bot_right_y - top_left_y;

      cv::rectangle(debug_bms_img_color, cv::Point2i(top_left_x,top_left_y),
          cv::Point2i(bot_right_x,bot_right_y), CV_RGB(255,0,0), 10);

      sub_img = cv_ptr->image(Range(top_left_y, bot_right_y), Range(top_left_x, bot_right_x));
      sub_mask = debug_bms_img_(Range(top_left_y, bot_right_y), Range(top_left_x, bot_right_x));

      if (cv::countNonZero(sub_mask) == 0) {
        continue;
      }
      samplereturn_msgs::Patch p_msg;
      p_msg.header = msg->header;
      p_msg.image = *(cv_bridge::CvImage(msg->header,"rgb8",sub_img).toImageMsg());
      p_msg.mask = *(cv_bridge::CvImage(msg->header,"mono8",sub_mask).toImageMsg());
      p_msg.image_roi.x_offset = top_left_x;
      p_msg.image_roi.y_offset = top_left_y;
      p_msg.image_roi.height = height;
      p_msg.image_roi.width = width;
      p_msg.cam_info = *cam_info;
      pa_msg.patch_array.push_back(p_msg);

    }
    ROS_DEBUG("End publish loop");
    pub_patch_array.publish(pa_msg);

    sensor_msgs::ImagePtr debug_img_msg =
      cv_bridge::CvImage(msg->header,"rgb8",debug_bms_img_color).toImageMsg();
    pub_bms_img.publish(debug_img_msg);

    ROS_DEBUG("messageCallback ended");
    saliency_mutex_.unlock();
  }

  /* Dynamic reconfigure callback */
  void configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level)
  {
      (void)level;
    saliency_mutex_.lock();
    ROS_DEBUG("configCallback");
    // Construct and configure BMS
    bms_ = BMS(config.bms_dilation_width_1, config.bms_opening_width,
        config.bms_normalize, config.bms_handle_border);
    bms_sample_step_ = config.bms_sample_step;
    bms_blur_std_ = config.bms_blur_std;
    bms_thresh_ = config.bms_thresh;
    bms_thresh_on_ = config.bms_thresh_on;
    bms_top_trim_ = config.bms_top_trim;
    bms_img_width_ = config.bms_img_width;

    // Configure and construct blob detector
    // Scale all spatialized params by bms image scale, since this operates on
    // the small image.
    blobDetect_on_ = config.blobDetect_on;

    blob_params_.minDistBetweenBlobs = config.minDistBetweenBlobs / scale_;

    blob_params_.filterByArea = config.filterByArea;
    blob_params_.minArea = config.minArea / pow(scale_,2);
    blob_params_.maxArea = config.maxArea / pow(scale_,2);

    blob_params_.filterByConvexity = config.filterByConvexity;
    blob_params_.minConvexity = config.minConvexity;
    blob_params_.maxConvexity = config.maxConvexity;

    blob_params_.minThreshold = config.minThreshold;
    blob_params_.maxThreshold = config.maxThreshold;
    blob_params_.thresholdStep = config.thresholdStep;
    blob_params_.minRepeatability = config.minRepeatability;

    blob_ = cv::SimpleBlobDetector::create(blob_params_);

    saliency_mutex_.unlock();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detector");
  SaliencyDetectorNode sd;
  ros::spin();
}
