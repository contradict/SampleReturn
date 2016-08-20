#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <samplereturn_msgs/PatchArray.h>
#include <samplereturn_msgs/NamedPoint.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/color_histogram_descriptor_paramsConfig.h>
#include <saliency_detector/colormodel.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/serialization/shared_ptr.hpp>

namespace saliency_detector {
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
  saliency_detector::color_histogram_descriptor_paramsConfig config_;

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
    enable_debug_ = (pub_debug_image.getNumSubscribers() != 0);
    if (msg->patch_array.empty()) {
      return;
    }
    samplereturn_msgs::PatchArray out_pa_msg;
    if (enable_debug_) {
      debug_image_ = cv::Mat::zeros(msg->cam_info.height,
          msg->cam_info.width, CV_8UC3);
    }
    for (size_t i = 0; i < msg->patch_array.size(); i++) {
      cv_bridge::CvImageConstPtr cv_ptr_mask, cv_ptr_img;
      sensor_msgs::ImageConstPtr msg_mask(&(msg->patch_array[i].mask), boost::serialization::null_deleter());
      sensor_msgs::ImageConstPtr msg_img(&(msg->patch_array[i].image), boost::serialization::null_deleter());
      try {
        cv_ptr_mask = cv_bridge::toCvShare(msg_mask, "mono8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge mask exception: %s", e.what());
      }
      try {
        cv_ptr_img = cv_bridge::toCvShare(msg_img, "rgb8");
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
      ColorModel cm(cv_ptr_img->image, cv_ptr_mask->image);
      HueHistogram hh_inner = cm.getInnerHueHistogram(config_.min_color_saturation, config_.low_saturation_limit, config_.high_saturation_limit);
      HueHistogram hh_outer = cm.getOuterHueHistogram(config_.min_color_saturation, config_.low_saturation_limit, config_.high_saturation_limit);
      double distance = hh_inner.distance(hh_outer);
    
      if (enable_debug_) {
          int x,y,w,h;
          x = msg->patch_array[i].image_roi.x_offset;
          y = msg->patch_array[i].image_roi.y_offset;
          w = msg->patch_array[i].image_roi.width;
          h = msg->patch_array[i].image_roi.height;
          char *str=hh_inner.str();
          if (y > 100) {
              cv::putText(debug_image_, str, cv::Point2d(x, y),
                      cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(255,0,0),4,cv::LINE_8);
          }
          else {
              cv::putText(debug_image_, str, cv::Point2d(x, y + h),
                      cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(255,0,0),4,cv::LINE_8);
          }
          free(str);
          hh_inner.draw_histogram(debug_image_, x, y+w);
      }

      // Compare inner and outer hists. If they're insufficiently different,
      // count this as a falsely salient positive.
      if (distance < config_.min_inner_outer_distance) {
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
          char bgdist[100];
          snprintf(bgdist, 100, "Too similar to bg: %4.3f", distance);
          if (y > 100) {
            cv::putText(debug_image_,bgdist,cv::Point2d(x, y-50),
               cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(255,0,0),4,cv::LINE_8);
          }
          else {
            cv::putText(debug_image_,bgdist,cv::Point2d(x, y + h),
               cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(255,0,0),4,cv::LINE_8);
          }
        }
        continue;
      }

      // Check inner hist against targets, either well-saturated in the specified
      // hue range, or unsaturated with a high value (metal and pre-cached)
      std::vector<std::tuple<double, double>> edges;
      edges.push_back(std::make_tuple(0, config_.min_target_hue));
      edges.push_back(std::make_tuple(config_.max_target_hue, 180));
      HueHistogram hh_colored_sample = ColorModel::getColoredSampleModel(edges, config_.low_saturation_limit, config_.high_saturation_limit);
      HueHistogram hh_value_sample = ColorModel::getValuedSampleModel(config_.low_saturation_limit, config_.high_saturation_limit);
      double hue_exemplar_distance = hh_colored_sample.distance(hh_inner);
      double value_exemplar_distance = hh_value_sample.distance(hh_inner);
      bool is_sample = (hue_exemplar_distance<config_.max_exemplar_distance) ||
                       (value_exemplar_distance<config_.max_exemplar_distance);
      if(enable_debug_)
      {
          int x,y,h;
          x = msg->patch_array[i].image_roi.x_offset;
          y = msg->patch_array[i].image_roi.y_offset;
          h = msg->patch_array[i].image_roi.height;
          char edist[100];
          snprintf(edist, 100, "h:%4.3f v:%4.3f", hue_exemplar_distance, value_exemplar_distance);
          cv::putText(debug_image_,edist,cv::Point2d(x+70, y + h + 50),
                  cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(255,0,0),4,cv::LINE_8);
      }
      // Maybe check outer hist against known background?
      if (is_sample)
      {
        samplereturn_msgs::NamedPoint np_msg;
        np_msg.header.stamp = msg->header.stamp;
        //np_msg.header.frame_id = "odom";
        np_msg.header.frame_id = msg->patch_array[i].world_point.header.frame_id;
        np_msg.point = msg->patch_array[i].world_point.point;
        hh_inner.to_msg(&np_msg.model.hue);
        np_msg.sensor_frame = msg->header.frame_id;
        pub_named_point.publish(np_msg);
      }
      else if(enable_debug_)
      {
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
      }
    }
    if (enable_debug_) {
      sensor_msgs::ImagePtr debug_image_msg =
        cv_bridge::CvImage(msg->header,"rgb8",debug_image_).toImageMsg();
      pub_debug_image.publish(debug_image_msg);
    }
  }

  void configCallback(saliency_detector::color_histogram_descriptor_paramsConfig &config, uint32_t level)
  {
      (void)level;
      config_ = config;
  }

};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_histogram_desciptor");
  saliency_detector::ColorHistogramDescriptorNode ch;
  ros::spin();
}
