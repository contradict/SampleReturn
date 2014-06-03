//#include "saliency_detector_core.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <samplereturn_msgs/NamedPoint.h>

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
  string img_topic;
  string bms_debug_topic;
  string sub_debug_topic;
  string sub_mask_debug_topic;
  string named_point_topic;
  string sub_camera_info_topic;

  boost::mutex saliency_mutex_;

  cv::Mat debug_bms_img_;
  BMS bms_;
  cv::SimpleBlobDetector::Params blob_params_;
  cv::SimpleBlobDetector blob_;
  bool blobDetect_on_;
  int bms_sample_step_;
  double bms_blur_std_;
  int bms_thresh_;
  bool bms_thresh_on_;
  double bms_top_trim_;
  double bms_img_width_;

  image_geometry::PinholeCameraModel cam_model_;
  cv::Mat inv_K_;

  ColorNaming cn_;

  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;

  public:
  SaliencyDetectorNode() {

    dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&SaliencyDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("img_topic", img_topic, string("/cameras/search/image"));
    private_node_handle_.param("bms_debug_topic", bms_debug_topic, string("bms_img"));
    private_node_handle_.param("sub_debug_topic", sub_debug_topic, string("sub_img"));
    private_node_handle_.param("sub_mask_debug_topic", sub_mask_debug_topic, string("sub_mask"));
    private_node_handle_.param("named_point_topic", named_point_topic, string("named_point"));
    private_node_handle_.param("camera_info_topic", sub_camera_info_topic, string("/cameras/search/info"));

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
      nh.advertise<samplereturn_msgs::NamedPoint>(named_point_topic.c_str(), 3);

    blob_params_.blobColor = 255;
    blob_params_.minArea = 15;
    blob_params_.maxArea = 600;
    blob_params_.filterByColor = true;
    blob_params_.filterByArea = true;
    blob_params_.filterByCircularity = false;
    blob_params_.filterByConvexity = false;
    blob_params_.filterByInertia = false;
    blob_params_.minDistBetweenBlobs = 50.;
    blob_ = cv::SimpleBlobDetector(blob_params_);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    ROS_INFO("Camera Info Callback");
    cam_model_.fromCameraInfo(msg);
    cv::Mat K = cam_model_.intrinsicMatrix();
    inv_K_ = K.inv();
  }

  void messageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    saliency_mutex_.lock();
    ROS_INFO("messageCallback");
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
      ROS_INFO("Thresholding");
      cv::threshold(debug_bms_img_, debug_bms_img_, bms_thresh_, 255, cv::THRESH_BINARY);
    }

    cv::Mat debug_bms_img_color;

    vector<cv::KeyPoint> kp;

    if (blobDetect_on_) {
      cv::Mat blob_copy;
      cv::threshold(debug_bms_img_, blob_copy, 30, 255, cv::THRESH_BINARY);
      blob_.detect(blob_copy, kp);
      ROS_INFO("Keypoints Detected: %lu", kp.size());
      cv::cvtColor(debug_bms_img_, debug_bms_img_color, CV_GRAY2RGB);
      for (size_t i=0; i < kp.size(); i++)
      {
        cv::circle(debug_bms_img_color, kp[i].pt, 3*kp[i].size, CV_RGB(255,0,0));
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

      string dominant_color = cn_.getDominantColor(interiorColor);
      std::cout << "Dominant color " << dominant_color << std::endl;
      string dominant_exterior_color = cn_.getDominantColor(exteriorColor);
      std::cout << "Dominant exterior color " << dominant_exterior_color << std::endl;

      if (cam_model_.initialized()
          && dominant_color != "green" && dominant_color != "brown"
          && dominant_exterior_color == "green"
          && dominant_color != "gray") {
        //float scale = cv_ptr->image.rows/600.;
        float scale = cv_ptr->image.cols/bms_img_width_;
        cv::Point3d ray =
          //cam_model_.projectPixelTo3dRay(cv::Point2d((kp[i].pt.x*scale+bms_top_trim_),kp[i].pt.y*scale));
          cam_model_.projectPixelTo3dRay(cv::Point2d(kp[i].pt.x*scale,(kp[i].pt.y*scale+bms_top_trim_)));
        std::cout << "BMS Coords: x: "<<kp[i].pt.x<<" y: "<<kp[i].pt.y<<std::endl;
        std::cout << "Pixel coords: x:" << kp[i].pt.x*scale <<"y: " << kp[i].pt.y*scale+bms_top_trim_ << std::endl;
        //std_msgs::Header header;
        //header.frame_id = "/search_camera_lens";
        samplereturn_msgs::NamedPoint np_msg;
        np_msg.header = msg->header;
        np_msg.header.frame_id = "/search_camera_lens";
        np_msg.name = dominant_color;
        np_msg.point.x = ray.x;
        np_msg.point.y = ray.y;
        np_msg.point.z = ray.z;
        pub_named_point.publish(np_msg);
      }
    }

    //if (kp.size() != 0) {
    //if (kp[1].pt.x > 30 && kp[1].pt.y > 30 && kp[1].pt.x < 500 && kp[1].pt.y < 350) {
    //  sub_img = small(Range(kp[1].pt.y-2*kp[1].size,kp[1].pt.y+2*kp[1].size),Range(kp[1].pt.x-2*kp[1].size,kp[1].pt.x+2*kp[1].size));
    //  sub_mask = debug_bms_img_(Range(kp[1].pt.y-2*kp[1].size,kp[1].pt.y+2*kp[1].size),Range(kp[1].pt.x-2*kp[1].size,kp[1].pt.x+2*kp[1].size));
    //  cv::threshold(sub_mask, sub_mask, 30, 255, cv::THRESH_BINARY);
    //  Eigen::Matrix<float,11,1> interiorColor(Eigen::Matrix<float,11,1>::Zero());
    //  Eigen::Matrix<float,11,1> exteriorColor(Eigen::Matrix<float,11,1>::Zero());
    //  exteriorColor = cn_.computeExteriorColor(sub_img,sub_mask);
    //  interiorColor = cn_.computeInteriorColor(sub_img,sub_mask,exteriorColor);

    //  std::cout << "Exterior Color" << std::endl;
    //  std::cout << exteriorColor << std::endl;
    //  std::cout << "Interior Color" << std::endl;
    //  std::cout << interiorColor << std::endl;

    //  string dominant_color = cn_.getDominantColor(interiorColor);
    //  std::cout << "Dominant color " << dominant_color << std::endl;

    //  std_msgs::Header sub_header;
    //  sensor_msgs::ImagePtr sub_img_msg = cv_bridge::CvImage(sub_header,"rgb8",sub_img).toImageMsg();
    //  pub_sub_img.publish(sub_img_msg);

    //  sensor_msgs::ImagePtr sub_mask_msg = cv_bridge::CvImage(sub_header,"mono8",sub_mask).toImageMsg();
    //  pub_sub_mask.publish(sub_mask_msg);
    //}
    //}

    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",debug_bms_img_color).toImageMsg();
    pub_bms_img.publish(debug_img_msg);

    ROS_INFO("messageCallback ended");
    saliency_mutex_.unlock();
  }

  /* Dynamic reconfigure callback */
  void configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level)
  {
    saliency_mutex_.lock();
    ROS_INFO("configCallback");
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

    saliency_mutex_.unlock();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detector");
  SaliencyDetectorNode sd;
  ros::spin();
}
