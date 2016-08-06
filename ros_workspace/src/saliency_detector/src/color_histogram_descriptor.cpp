#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <samplereturn_msgs/PatchArray.h>
#include <samplereturn_msgs/NamedPoint.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/color_histogram_descriptor_paramsConfig.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Take in a PatchArray, compute a color model of the salient object in each patch,
// publish a NamedPoint. This is the end of the detection part of sample search,
// objects past here have matched saliency, shape, size, and color criteria.
class ColorHistogramDescriptorNode
{
  ros::Subscriber sub_patch_array;
  ros::Publisher pub_named_point;
  ros::Publisher pub_debug_image;
  std::string sub_patch_array_topic;
  std::string pub_named_point_topic;
  std::string pub_debug_image_topic;

  dynamic_reconfigure::Server<saliency_detector::color_histogram_descriptor_paramsConfig> dr_srv;

  // OpenCV HSV represents H between 0-180, remember that
  int min_target_hue_;
  int max_target_hue_;
  int min_color_saturation_;
  // Acknowledge that measurements are noisy, put some slop into stated ranges
  int hue_slop_;
  // Maximum correlation allowed between inner and outer regions of patches
  double max_inner_outer_hist_correl_;

  bool enable_debug_;
  cv::Mat debug_image_;

  public:
  ColorHistogramDescriptorNode() {
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<saliency_detector::color_histogram_descriptor_paramsConfig>::CallbackType cb;
    cb = boost::bind(&ColorHistogramDescriptorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("sub_patch_array_topic", sub_patch_array_topic,
        std::string("projected_patch_array"));
    private_node_handle_.param("pub_named_point_topic", pub_named_point_topic,
        std::string("named_point"));
    private_node_handle_.param("pub_debug_image_topic", pub_debug_image_topic,
        std::string("color_debug_image"));

    sub_patch_array =
      nh.subscribe(sub_patch_array_topic, 1, &ColorHistogramDescriptorNode::patchArrayCallback, this);

    pub_named_point =
      nh.advertise<samplereturn_msgs::NamedPoint>(pub_named_point_topic.c_str(), 1);

    pub_debug_image =
      nh.advertise<sensor_msgs::Image>(pub_debug_image_topic.c_str(), 1);

    enable_debug_ = false;
  }

  void patchArrayCallback(const samplereturn_msgs::PatchArrayConstPtr& msg)
  {
    if (msg->patch_array.empty()) {
      return;
    }
    samplereturn_msgs::PatchArray out_pa_msg;
    if (enable_debug_) {
      debug_image_ = cv::Mat::ones(msg->patch_array[0].cam_info.height,
          msg->patch_array[0].cam_info.width, CV_8UC3)*255;
    }
    cv_bridge::CvImagePtr cv_ptr_mask, cv_ptr_img;
    for (int i = 0; i < msg->patch_array.size(); i++) {
      try {
        cv_ptr_mask = cv_bridge::toCvCopy(msg->patch_array[i].mask, "mono8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge mask exception: %s", e.what());
      }
      try {
        cv_ptr_img = cv_bridge::toCvCopy(msg->patch_array[i].image, "rgb8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge image exception: %s", e.what());
      }
      if (enable_debug_) {
        cv_ptr_img->image.copyTo(debug_image_(cv::Rect(msg->patch_array[i].image_roi.x_offset,
                msg->patch_array[i].image_roi.y_offset,
                msg->patch_array[i].image_roi.width,
                msg->patch_array[i].image_roi.height)));
      }
      // Use mask to get background color and foreground color
      cv::Mat hsv, inner_mask, outer_mask, saturation_mask, saturation;
      cv::cvtColor(cv_ptr_img->image, hsv, cv::COLOR_RGB2HSV);
      cv::erode(cv_ptr_mask->image, inner_mask, cv::Mat());
      cv::erode(255 - (cv_ptr_mask->image), outer_mask, cv::Mat());
      cv::extractChannel(hsv, saturation, 1);
      cv::threshold(saturation, saturation_mask, min_color_saturation_, 255, cv::THRESH_BINARY);
      cv::bitwise_or(inner_mask, saturation_mask, inner_mask);
      cv::bitwise_or(outer_mask, saturation_mask, outer_mask);

      int hbins = 60;
      int histSize[] = { hbins };
      float hrange[] = { 0, 180 };
      const float* ranges[] = { hrange };
      int channels[] = { 0 };
      cv::MatND inner_hist, outer_hist;
      cv::calcHist(&hsv, 1, channels, inner_mask, inner_hist, 1,
          histSize, ranges , true, false);
      cv::calcHist(&hsv, 1, channels, outer_mask, outer_hist, 1,
          histSize, ranges , true, false);
      //Normalize histograms to account for different size
      cv::normalize(inner_hist, inner_hist, 1.0, 0.0, cv::NORM_MINMAX);
      cv::normalize(outer_hist, outer_hist, 1.0, 0.0, cv::NORM_MINMAX);

      // Compare inner and outer hists. If they're insufficiently different,
      // count this as a falsely salient positive.
      double hist_correl = cv::compareHist(inner_hist, outer_hist, cv::HISTCMP_CORREL);
      if (hist_correl > max_inner_outer_hist_correl_) {
        // Nix region, with explanatory text
        if (enable_debug_) {
          int x,y,w,h;
          x = msg->patch_array[i].image_roi.x_offset;
          y = msg->patch_array[i].image_roi.y_offset;
          w = msg->patch_array[i].image_roi.width;
          h = msg->patch_array[i].image_roi.height;
          cv::line(debug_image_,
              cv::Point2f(x, y),
              cv::Point2f(x + w, y + h), cv::Scalar(255,0,0), 20);
          cv::line(debug_image_,
              cv::Point2f(x + w, y),
              cv::Point2f(x, y + h), cv::Scalar(255,0,0), 20);
          if (y > 100) {
            cv::putText(debug_image_,"Too Similar to BG",cv::Point2d(x, y),
               cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,0,0),4,cv::LINE_8,true);
          }
          else {
            cv::putText(debug_image_,"Too Similar to BG",cv::Point2d(x, y + h),
               cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,0,0),4,cv::LINE_8,false);
          }
        }
        continue;
      }

      // Check inner hist against targets, either well-saturated in the specified
      // hue range, or unsaturated with a high value (metal and pre-cached)
      double min_hue, max_hue;
      int min_hue_loc, max_hue_loc;
      cv::minMaxIdx(inner_hist, &min_hue, &max_hue, &min_hue_loc, &max_hue_loc);
      double dominant_hue = max_hue_loc * (180/hbins);
      if ((dominant_hue < max_target_hue_) && (dominant_hue > min_target_hue_)) {
        samplereturn_msgs::NamedPoint np_msg;
        np_msg.header.stamp = msg->patch_array[i].header.stamp;
        //np_msg.header.frame_id = "odom";
        np_msg.header.frame_id = msg->patch_array[i].world_point.header.frame_id;
        np_msg.point = msg->patch_array[i].world_point.point;
        np_msg.hue = dominant_hue;
        pub_named_point.publish(np_msg);
      }
      else {
        if (enable_debug_) {
          int x,y,w,h;
          x = msg->patch_array[i].image_roi.x_offset;
          y = msg->patch_array[i].image_roi.y_offset;
          w = msg->patch_array[i].image_roi.width;
          h = msg->patch_array[i].image_roi.height;
          cv::line(debug_image_,
              cv::Point2f(x, y),
              cv::Point2f(x + w, y + h), cv::Scalar(255,0,0), 20);
          cv::line(debug_image_,
              cv::Point2f(x + w, y),
              cv::Point2f(x, y + h), cv::Scalar(255,0,0), 20);
          if (y > 100) {
            cv::putText(debug_image_,"Hue Out of Range",cv::Point2d(x, y),
               cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,0,0),4,cv::LINE_8,true);
          }
          else {
            cv::putText(debug_image_,"Hue Out of Range",cv::Point2d(x, y + h),
               cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,0,0),4,cv::LINE_8,false);
          }
        }
      }

      // Maybe check outer hist against known background?
    }
    if (enable_debug_) {
      sensor_msgs::ImagePtr debug_image_msg =
        cv_bridge::CvImage(msg->patch_array[0].header,"rgb8",debug_image_).toImageMsg();
      pub_debug_image.publish(debug_image_msg);
    }
  }

  void configCallback(saliency_detector::color_histogram_descriptor_paramsConfig &config, uint32_t level)
  {
    min_target_hue_ = config.min_target_hue;
    max_target_hue_ = config.max_target_hue;
    hue_slop_ = config.hue_slop;
    enable_debug_ = config.enable_debug;
  }

  double hueDistance(double hue, double hue_ref)
  {
    if (abs(hue - hue_ref) <= 90) {
      return abs(hue - hue_ref);
    }
    else {
      return 180 - abs(hue - hue_ref);
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_histogram_desciptor");
  ColorHistogramDescriptorNode ch;
  ros::spin();
}
