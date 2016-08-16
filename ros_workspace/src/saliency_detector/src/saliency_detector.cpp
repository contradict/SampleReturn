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

  boost::mutex saliency_mutex_;

  BMS bms_;

  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;
  saliency_detector::saliency_detector_paramsConfig config_;

  public:
  SaliencyDetectorNode() {
    ros::NodeHandle nh;

    it = new image_transport::ImageTransport(nh);

    dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&SaliencyDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");

    sub_img =
      it->subscribeCamera("image", 10, &SaliencyDetectorNode::messageCallback, this);

    pub_bms_img =
      nh.advertise<sensor_msgs::Image>("bms_img", 3);

    pub_patch_array =
      nh.advertise<samplereturn_msgs::PatchArray>("patch_array", 3);

  }

  void messageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    saliency_mutex_.lock();

    bool have_debug_listener = pub_bms_img.getNumSubscribers()>0;

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
    double scale = cv_ptr->image.cols/config_.bms_img_width;
    cv::resize(cv_ptr->image.rowRange(config_.bms_top_trim,cv_ptr->image.rows),
        small,
        cv::Size(config_.bms_img_width,
            (cv_ptr->image.rows-config_.bms_top_trim)/scale),
        0.0,0.0,cv::INTER_NEAREST);
    ROS_DEBUG("End resize");

    // Compute saliency map
    ROS_DEBUG("Begin BMS Comp");
    bms_.computeSaliency(small, config_.bms_sample_step);
    cv::Mat saliency_map = bms_.getSaliencyMap();
    ROS_DEBUG("End BMS Comp");

    // create blob detector with current scale and config
    cv::Ptr<cv::SimpleBlobDetector> blob = createBlob(scale);

    // Allocate output image and detected region structures
    std::vector<cv::KeyPoint> kp;

    // Detect blobs matching criteria
    blob->detect(saliency_map, kp);
    ROS_INFO("Keypoints Detected: %lu", kp.size());
    ROS_DEBUG("End Blob Comp");

    // Scale back up
    ROS_DEBUG("Begin resize");
    cv::Mat saliency_map_large;
    cv::resize(saliency_map, saliency_map_large,
            cv::Size(cv_ptr->image.cols,cv_ptr->image.rows),
            0.0, 0.0, cv::INTER_NEAREST);
    cv::Mat debug_bms_img_color;
    if(have_debug_listener)
    {
        cv::cvtColor(saliency_map_large, debug_bms_img_color, CV_GRAY2RGB);
    }
    ROS_DEBUG("End resize");

    // Allocate images and mask for patch publishing
    cv::Mat sub_img;
    cv::Mat sub_mask;
    samplereturn_msgs::PatchArray pa_msg;

    // Scale keypoint params back up from smaller BMS image
    ROS_DEBUG("Begin publish loop");
    for (size_t i = 0; i < kp.size(); i++) {
      int x = kp[i].pt.x * scale;
      int y = kp[i].pt.y * scale;
      // Pad a bit to avoid clipping
      int size = config_.patch_scaling_factor * kp[i].size * scale;
      int top_left_x = max(x-size/2,0);
      int top_left_y = max(y-size/2,0);
      int bot_right_x = min(x+size/2,cv_ptr->image.cols - 1);
      int bot_right_y = min(y+size/2,cv_ptr->image.rows - 1);
      int width = bot_right_x - top_left_x;
      int height = bot_right_y - top_left_y;

      if(have_debug_listener)
      {
          cv::rectangle(debug_bms_img_color, cv::Point2i(top_left_x,top_left_y),
              cv::Point2i(bot_right_x,bot_right_y), CV_RGB(255,0,0), 10);
      }

      sub_img = cv_ptr->image(Range(top_left_y, bot_right_y), Range(top_left_x, bot_right_x));
      sub_mask = saliency_map_large(Range(top_left_y, bot_right_y), Range(top_left_x, bot_right_x));

      // Threshold grayscale saliency into binary
      if (config_.bms_thresh_on) {
        ROS_DEBUG("Thresholding");
        // Compute threshold for each segmented blob
        double minVal, maxVal, thresh;
        cv::minMaxLoc(sub_mask, &minVal, &maxVal);
        thresh = minVal + config_.bms_thresh_fraction * (maxVal - minVal);
        cv::threshold(sub_mask, sub_mask, thresh, 255, cv::THRESH_BINARY);
        cv::Mat colormask;
        cv::cvtColor(sub_mask, colormask, CV_GRAY2RGB);
      }

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

    if(have_debug_listener)
    {
        sensor_msgs::ImagePtr debug_img_msg =
            cv_bridge::CvImage(msg->header,"rgb8",debug_bms_img_color).toImageMsg();
        pub_bms_img.publish(debug_img_msg);
    }

    ROS_DEBUG("messageCallback ended");
    saliency_mutex_.unlock();
  }

  cv::Ptr<cv::SimpleBlobDetector>
  createBlob(double scale)
  {
      cv::SimpleBlobDetector::Params blob_params;
      blob_params.filterByCircularity = false;
      blob_params.filterByInertia = false;
      blob_params.minDistBetweenBlobs = config_.minDistBetweenBlobs/scale;

      blob_params.filterByColor = true;
      blob_params.blobColor = 255;

      blob_params.filterByArea = config_.filterByArea;
      blob_params.minArea = config_.minArea/scale/scale;
      blob_params.maxArea = config_.maxArea/scale/scale;

      blob_params.filterByConvexity = config_.filterByConvexity;
      blob_params.minConvexity = config_.minConvexity;
      blob_params.maxConvexity = config_.maxConvexity;

      blob_params.minThreshold = config_.minThreshold;
      blob_params.maxThreshold = config_.maxThreshold;
      blob_params.thresholdStep = config_.thresholdStep;
      blob_params.minRepeatability = config_.minRepeatability;

      return cv::SimpleBlobDetector::create(blob_params);
  }

  /* Dynamic reconfigure callback */
  void configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level)
  {
    (void)level;
    saliency_mutex_.lock();
    // Construct and configure BMS
    config_ = config;
    bms_ = BMS(config.bms_dilation_width_1, config.bms_opening_width,
        config.bms_normalize, config.bms_handle_border);


    saliency_mutex_.unlock();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detector");
  SaliencyDetectorNode sd;
  ros::spin();
}
