//#include "saliency_detector_core.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <samplereturn_msgs/NamedPoint.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/saliency_detector_paramsConfig.h>

#include "BMS.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <Eigen/Dense>
#include "color_naming.h"

class SaliencyDetectorNode
{
  ros::NodeHandle nh;
  ros::Subscriber sub_img;
  ros::Subscriber sub_camera_info;
  ros::Publisher pub_bms_img;
  ros::Publisher pub_sub_img;
  ros::Publisher pub_sub_mask;
  ros::Publisher pub_named_point;
  std::string img_topic;
  std::string bms_debug_topic;
  std::string sub_debug_topic;
  std::string sub_mask_debug_topic;
  std::string named_point_topic;
  std::string sub_camera_info_topic;

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

  bool filter_by_real_area_;
  double min_real_area_;
  double max_real_area_;
  double min_real_area_redpinkpurple_;
  double max_real_area_redpinkpurple_;
  double min_real_area_bluewhitegray_;
  double max_real_area_bluewhitegray_;
  double min_real_area_yellow_;
  double max_real_area_yellow_;

  XmlRpc::XmlRpcValue interior_colors_, exterior_colors_;
  std::vector<std::string> interior_colors_vec_, exterior_colors_vec_;

  image_geometry::PinholeCameraModel cam_model_;
  cv::Mat inv_K_;

  ColorNaming cn_;

  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;

  tf::TransformListener listener_;

  public:
  SaliencyDetectorNode() {

    dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&SaliencyDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("img_topic", img_topic, std::string("/cameras/search/image"));
    private_node_handle_.param("bms_debug_topic", bms_debug_topic, std::string("bms_img"));
    private_node_handle_.param("sub_debug_topic", sub_debug_topic, std::string("sub_img"));
    private_node_handle_.param("sub_mask_debug_topic", sub_mask_debug_topic, std::string("sub_mask"));
    private_node_handle_.param("named_point_topic", named_point_topic, std::string("named_point"));
    private_node_handle_.param("camera_info_topic", sub_camera_info_topic, std::string("/cameras/search/info"));

    private_node_handle_.getParam("interior_colors",interior_colors_);
    private_node_handle_.getParam("exterior_colors",exterior_colors_);

    for (int i=0; i<interior_colors_.size(); i++) {
      interior_colors_vec_.push_back(static_cast<std::string>(interior_colors_[i]));
    }
    for (int i=0; i<exterior_colors_.size(); i++) {
      exterior_colors_vec_.push_back(static_cast<std::string>(exterior_colors_[i]));
    }

    sub_img =
      nh.subscribe(img_topic.c_str(), 3, &SaliencyDetectorNode::messageCallback, this);

    sub_camera_info =
      nh.subscribe(sub_camera_info_topic.c_str(), 3, &SaliencyDetectorNode::cameraInfoCallback, this);

    pub_bms_img =
      nh.advertise<sensor_msgs::Image>(bms_debug_topic.c_str(), 3);

    pub_sub_img =
      nh.advertise<sensor_msgs::Image>(sub_debug_topic.c_str(), 3);

    pub_sub_mask =
      nh.advertise<sensor_msgs::Image>(sub_mask_debug_topic.c_str(), 3);

    pub_named_point =
      nh.advertise<samplereturn_msgs::NamedPoint>(named_point_topic.c_str(), 12);

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

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    ROS_DEBUG("Camera Info Callback");
    cam_model_.fromCameraInfo(msg);
    cv::Mat K = cv::Mat(cam_model_.intrinsicMatrix());
    inv_K_ = K.inv();
  }

  void messageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    cv::Mat small;
    cv::resize(cv_ptr->image.rowRange(bms_top_trim_,cv_ptr->image.rows),
        small,cv::Size(bms_img_width_,(cv_ptr->image.rows-bms_top_trim_)*(bms_img_width_/cv_ptr->image.cols)),
        0.0,0.0,cv::INTER_AREA);

    bms_.computeSaliency(small, bms_sample_step_);
    debug_bms_img_ = bms_.getSaliencyMap().clone();

    if (bms_thresh_on_) {
      ROS_DEBUG("Thresholding");
      cv::threshold(debug_bms_img_, debug_bms_img_, bms_thresh_, 255, cv::THRESH_BINARY);
    }

    cv::Mat debug_bms_img_color;

    std::vector<cv::KeyPoint> kp;

    if (blobDetect_on_) {
      cv::Mat blob_copy = debug_bms_img_.clone();
      blob_->detect(blob_copy, kp);
      ROS_DEBUG("Keypoints Detected: %lu", kp.size());
      cv::cvtColor(debug_bms_img_, debug_bms_img_color, CV_GRAY2RGB);
      for (size_t i=0; i < kp.size(); i++)
      {
        cv::circle(debug_bms_img_color, kp[i].pt, 3*kp[i].size, CV_RGB(255,0,0), 1, 4);
      }
    }

    cv::Mat sub_img;
    cv::Mat sub_mask;

    for (int i = 0; i < kp.size(); i++) {
      int x = kp[i].pt.x;
      int y = kp[i].pt.y;
      int size = 2*kp[i].size;
      sub_img = small(Range(max(y-size,0), min(y+size,small.rows)), Range(max(x-size,0), min(x+size,small.cols)));
      sub_mask = debug_bms_img_(Range(max(y-size,0), min(y+size,small.rows)), Range(max(x-size,0), min(x+size,small.cols)));
      if (cv::countNonZero(sub_mask) == 0) {
        continue;
      }
      Eigen::Matrix<float,11,1> interiorColor(Eigen::Matrix<float,11,1>::Zero());
      Eigen::Matrix<float,11,1> exteriorColor(Eigen::Matrix<float,11,1>::Zero());
      exteriorColor = cn_.computeExteriorColor(sub_img,sub_mask);
      interiorColor = cn_.computeInteriorColor(sub_img,sub_mask,exteriorColor);

      std::string dominant_color = cn_.getDominantColor(interiorColor);
      std::cout << "Dominant color " << dominant_color << std::endl;
      std::string dominant_exterior_color = cn_.getDominantColor(exteriorColor);
      std::cout << "Dominant exterior color " << dominant_exterior_color << std::endl;

      cv::putText(debug_bms_img_color, dominant_color, kp[i].pt, FONT_HERSHEY_SIMPLEX, 0.5,
          CV_RGB(100,100,100));

      std::vector<std::string>::iterator in_it, ex_it;
      in_it = std::find(interior_colors_vec_.begin(),interior_colors_vec_.end(),dominant_color);
      ex_it = std::find(exterior_colors_vec_.begin(),exterior_colors_vec_.end(),dominant_exterior_color);

      if (cam_model_.initialized()
          && in_it != interior_colors_vec_.end()
          && ex_it != exterior_colors_vec_.end()){
        float scale = cv_ptr->image.cols/bms_img_width_;
        cv::Point3d ray =
          cam_model_.projectPixelTo3dRay(cv::Point2d(kp[i].pt.x*scale,(kp[i].pt.y*scale+bms_top_trim_)));
        if(!checkContourSize(sub_mask,ray,scale,msg->header,dominant_color))
            continue;
        cv::circle(debug_bms_img_color, kp[i].pt, 2*kp[i].size, CV_RGB(0,0,255), 2, CV_AA);
        cv::putText(debug_bms_img_color, dominant_color, kp[i].pt, FONT_HERSHEY_SIMPLEX, 0.5,
            CV_RGB(0,255,0));
        std::cout << "BMS Coords: x: "<<kp[i].pt.x<<" y: "<<kp[i].pt.y<<std::endl;
        std::cout << "Pixel coords: x:" << kp[i].pt.x*scale <<"y: " << kp[i].pt.y*scale+bms_top_trim_ << std::endl;
        //std_msgs::Header header;
        //header.frame_id = "/search_camera_lens";
        samplereturn_msgs::NamedPoint np_msg;
        np_msg.header = msg->header;
        np_msg.name = dominant_color;
        np_msg.point.x = ray.x;
        np_msg.point.y = ray.y;
        np_msg.point.z = ray.z;
        pub_named_point.publish(np_msg);
      }
    }

    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",debug_bms_img_color).toImageMsg();
    pub_bms_img.publish(debug_img_msg);

    ROS_DEBUG("messageCallback ended");
    saliency_mutex_.unlock();
  }

  bool checkContourSize(const cv::Mat region, const cv::Point3d ray, float scale,
      const std_msgs::Header header, std::string color)
  {
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours( region, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
      // find largest contour
      double maxArea = 0;
      int max_idx = 0;
      for (int i=0; i<contours.size(); i++) {
        double area = cv::contourArea(cv::Mat(contours[i]));
        if (area > maxArea) {
          maxArea = area;
          max_idx = i;
        }
      }
      // project ray into ground, assuming flat ground
      tf::StampedTransform camera_transform;
      geometry_msgs::PointStamped camera_point, base_link_point;
      tf::Vector3 pos;
      camera_point.header = header;
      camera_point.point.x = ray.x;
      camera_point.point.y = ray.y;
      camera_point.point.z = ray.z;
      listener_.lookupTransform("base_link",header.frame_id,ros::Time(0),camera_transform);
      pos = camera_transform.getOrigin();
      camera_point.header.stamp = ros::Time(0);
      listener_.transformPoint("base_link",camera_point,base_link_point);

      float x_slope = (base_link_point.point.x - pos.getX())/(pos.getZ()-base_link_point.point.z);
      float y_slope = (base_link_point.point.y - pos.getY())/(pos.getZ()-base_link_point.point.z);

      float x = x_slope*pos.getZ();
      float y = y_slope*pos.getZ();
      float z = pos.getZ();
      float range = sqrt(x*x + y*y + z*z);

      float ang_per_px =
        scale*atan(cam_model_.fullResolution().width/cam_model_.fx())/cam_model_.fullResolution().width;
      float realArea = ang_per_px*ang_per_px*range*range*maxArea;
      // Convert from square meters to square cm for more human-readable numbers
      realArea *= 10000;

      if ((color == "red") || (color == "pink") || (color == "purple") || (color == "orange")) {
        if ((realArea < max_real_area_redpinkpurple_) && (realArea > min_real_area_redpinkpurple_)) {
          return true;
        }
        else {
          return false;
        }
      }
      if ((color == "blue") || (color == "white") || (color == "gray")) {
        if ((realArea < max_real_area_bluewhitegray_) && (realArea > min_real_area_bluewhitegray_)) {
          return true;
        }
        else {
          return false;
        }
      }
      if (color == "yellow") {
        if ((realArea < max_real_area_yellow_) && (realArea > min_real_area_yellow_)) {
          return true;
        }
        else {
          return false;
        }
      }
      if ((realArea < max_real_area_) && (realArea > min_real_area_)) {
        return true;
      }
      else {
        return false;
      }
  }

  /* Dynamic reconfigure callback */
  void configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level)
  {
    saliency_mutex_.lock();
    ROS_DEBUG("configCallback");
    // Construct BMS
    bms_ = BMS(config.bms_dilation_width_1, config.bms_opening_width,
        config.bms_normalize, config.bms_handle_border);

    bms_sample_step_ = config.bms_sample_step;
    bms_blur_std_ = config.bms_blur_std;
    bms_thresh_ = config.bms_thresh;
    bms_thresh_on_ = config.bms_thresh_on;
    bms_top_trim_ = config.bms_top_trim;
    bms_img_width_ = config.bms_img_width;

    blobDetect_on_ = config.blobDetect_on;

    blob_params_.minDistBetweenBlobs = config.minDistBetweenBlobs;

    blob_params_.filterByArea = config.filterByArea;
    blob_params_.minArea = config.minArea;
    blob_params_.maxArea = config.maxArea;

    blob_params_.filterByConvexity = config.filterByConvexity;
    blob_params_.minConvexity = config.minConvexity;
    blob_params_.maxConvexity = config.maxConvexity;

    blob_params_.minThreshold = config.minThreshold;
    blob_params_.maxThreshold = config.maxThreshold;
    blob_params_.thresholdStep = config.thresholdStep;
    blob_params_.minRepeatability = config.minRepeatability;

    blob_ = cv::SimpleBlobDetector::create(blob_params_);

    filter_by_real_area_ = config.filterbyRealArea;
    min_real_area_ = config.minRealArea;
    max_real_area_ = config.maxRealArea;

    min_real_area_redpinkpurple_ = config.minRealArea_redpinkpurple;
    max_real_area_redpinkpurple_ = config.maxRealArea_redpinkpurple;
    min_real_area_bluewhitegray_ = config.minRealArea_bluewhitegray;
    max_real_area_bluewhitegray_ = config.maxRealArea_bluewhitegray;
    min_real_area_yellow_ = config.minRealArea_yellow;
    max_real_area_yellow_ = config.maxRealArea_yellow;

    saliency_mutex_.unlock();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detector");
  SaliencyDetectorNode sd;
  ros::spin();
}
