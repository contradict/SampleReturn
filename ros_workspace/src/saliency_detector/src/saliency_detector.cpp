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
#include <thread>

struct index_score {
  float count;
  uchar R;
  uchar G;
  uchar B;
  uchar L;
  uchar a;
  uchar b;
  uchar H;
  uchar S;
  uchar V;
} __attribute__((packed));

class SaliencyDetectorNode
{
  ros::NodeHandle nh;
  ros::Subscriber sub_img;
  ros::Subscriber sub_camera_info;
  ros::Publisher pub_bms_img;
  ros::Publisher pub_gcs_img;
  ros::Publisher pub_sub_img;
  ros::Publisher pub_sub_mask;
  ros::Publisher pub_named_point;
  string img_topic;
  string bms_debug_topic;
  string gcs_debug_topic;
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

  XmlRpc::XmlRpcValue interior_colors_, exterior_colors_;
  std::vector<std::string> interior_colors_vec_, exterior_colors_vec_;

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
    private_node_handle_.param("gcs_debug_topic", gcs_debug_topic, string("gcs_img"));
    private_node_handle_.param("sub_debug_topic", sub_debug_topic, string("sub_img"));
    private_node_handle_.param("sub_mask_debug_topic", sub_mask_debug_topic, string("sub_mask"));
    private_node_handle_.param("named_point_topic", named_point_topic, string("named_point"));
    private_node_handle_.param("camera_info_topic", sub_camera_info_topic, string("/cameras/search/info"));

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

    pub_gcs_img =
      nh.advertise<sensor_msgs::Image>(gcs_debug_topic.c_str(), 3);

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
    blob_ = cv::SimpleBlobDetector(blob_params_);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    ROS_DEBUG("Camera Info Callback");
    cam_model_.fromCameraInfo(msg);
    cv::Mat K = cv::Mat(cam_model_.intrinsicMatrix());
    inv_K_ = K.inv();
  }

  // There may be a bug in this, it produces slightly different maps than
  // the python script example. Qualitatively very similar, but with slightly
  // different range. I will hopefully get back to finding and fixing this.
  void computeGlobalSaliency(cv::Mat image, cv::Mat& out) {
    cv::Mat lab_image, hsv_image;
    cv::cvtColor(image,lab_image,cv::COLOR_RGB2Lab);
    cv::cvtColor(image,hsv_image,cv::COLOR_RGB2HSV);

    int channels[] = {0,1,2};
    int hist_size[] = {12,12,12};
    float r_range[] = {0,256};
    float g_range[] = {0,256};
    float b_range[] = {0,256};
    const float* ranges[] = {r_range,g_range,b_range};

    cv::Mat hist;
    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 3, hist_size, ranges);

    float sum = 0;
    std::vector<index_score> top_indices;
    // Normalize hist
    for (int i=0; i<hist_size[0]; i++) {
      for (int j=0; j<hist_size[1]; j++) {
        for (int k=0; k<hist_size[2]; k++) {
          sum += hist.at<float>(i,j,k);
          float bin_width = r_range[1]/float(hist_size[0]);
          struct index_score i_s;
          i_s.count = hist.at<float>(i,j,k);
          i_s.R = (i+0.5)*bin_width;
          i_s.G = (j+0.5)*bin_width;
          i_s.B = (k*0.5)*bin_width;
          // Add Lab space for distance metric
          cv::Mat rgb(1,1,CV_8UC3,&i_s.R);
          cv::Mat Lab(1,1,CV_8UC3,&i_s.L);
          cv::Mat HSV(1,1,CV_8UC3,&i_s.H);
          cv::cvtColor(rgb,Lab,CV_RGB2Lab);
          cv::cvtColor(rgb,HSV,CV_RGB2HSV);
          top_indices.push_back(i_s);
        }
      }
    }
    // Reduce hist to n ~= 100 bins (try to cover some fraction of pixels. e.g.95%)
    std::sort(top_indices.begin(), top_indices.end(), [](struct index_score a, struct index_score b) {
        return b.count < a.count;
        });

    // For query image, iterate over hist bins to compute per-pixel score,
    // which is distance between query pixel and hist bin color * hist bin count
    //cv::Mat out = cv::Mat::zeros(image.rows,image.cols,CV_32FC1);
    for (int n=0; n<40; n++) {
      for (int i=0; i<image.rows;i++) {
        for (int j=0; j<image.cols;j++) {
          cv::Vec3b lab_px = lab_image.at<cv::Vec3b>(i,j);
          cv::Vec3b hsv_px = hsv_image.at<cv::Vec3b>(i,j);
          float diff = abs(int(hsv_px[0]) - int(top_indices[n].H))
            + abs(int(lab_px[1]) - int(top_indices[n].a)) + abs(int(lab_px[2]) - int(top_indices[n].b));
          out.at<float>(i,j) += top_indices[n].count*diff/sum;
        }
      }
    }
    cv::Mat thresh, dst;
    cv::threshold(out,out,110,255,cv::THRESH_BINARY);
    out.convertTo(out, CV_8U);
    //return dst;
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

    cv::Mat out = cv::Mat::zeros(small.rows,small.cols,CV_32FC1);
    //std::thread first(&SaliencyDetectorNode::computeGlobalSaliency,this,small,out);
    //std::thread second(&bms_.computeSaliency,this,small,bms_sample_step_);

    //first.join();
    //second.join();

    //cv::Mat out = cv::Mat::zeros(small.rows,small.cols,CV_32FC1);
    //cv::Mat global_saliency = computeGlobalSaliency(small, out);
    computeGlobalSaliency(small, out);
    std_msgs::Header header;
    //sensor_msgs::ImagePtr debug_img_msg;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"mono8",out).toImageMsg();
    //sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"mono8",global_saliency).toImageMsg();
    pub_gcs_img.publish(debug_img_msg);

    bms_.computeSaliency(small, bms_sample_step_);
    debug_bms_img_ = bms_.getSaliencyMap().clone();

    if (bms_thresh_on_) {
      ROS_DEBUG("Thresholding");
      cv::threshold(debug_bms_img_, debug_bms_img_, bms_thresh_, 255, cv::THRESH_BINARY);
    }

    cv::Mat debug_bms_img_color;

    vector<cv::KeyPoint> kp;

    if (blobDetect_on_) {
      cv::Mat blob_copy = debug_bms_img_.clone();
      blob_.detect(blob_copy, kp);
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

      string dominant_color = cn_.getDominantColor(interiorColor);
      std::cout << "Dominant color " << dominant_color << std::endl;
      string dominant_exterior_color = cn_.getDominantColor(exteriorColor);
      std::cout << "Dominant exterior color " << dominant_exterior_color << std::endl;

      cv::putText(debug_bms_img_color, dominant_color, kp[i].pt, FONT_HERSHEY_SIMPLEX, 0.5,
          CV_RGB(100,100,100));

      std::vector<std::string>::iterator in_it, ex_it;
      in_it = std::find(interior_colors_vec_.begin(),interior_colors_vec_.end(),dominant_color);
      ex_it = std::find(exterior_colors_vec_.begin(),exterior_colors_vec_.end(),dominant_exterior_color);

      if (cam_model_.initialized()
          && in_it != interior_colors_vec_.end()
          && ex_it != exterior_colors_vec_.end()){
        cv::circle(debug_bms_img_color, kp[i].pt, 2*kp[i].size, CV_RGB(0,0,255), 2, CV_AA);
        cv::putText(debug_bms_img_color, dominant_color, kp[i].pt, FONT_HERSHEY_SIMPLEX, 0.5,
            CV_RGB(0,255,0));
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
        np_msg.name = dominant_color;
        np_msg.point.x = ray.x;
        np_msg.point.y = ray.y;
        np_msg.point.z = ray.z;
        pub_named_point.publish(np_msg);
      }
    }

    //std_msgs::Header header;
    //sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",debug_bms_img_color).toImageMsg();
    debug_img_msg = cv_bridge::CvImage(header,"rgb8",debug_bms_img_color).toImageMsg();
    pub_bms_img.publish(debug_img_msg);

    ROS_DEBUG("messageCallback ended");
    saliency_mutex_.unlock();
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

    blob_ = cv::SimpleBlobDetector(blob_params_);

    saliency_mutex_.unlock();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detector");
  SaliencyDetectorNode sd;
  ros::spin();
}
