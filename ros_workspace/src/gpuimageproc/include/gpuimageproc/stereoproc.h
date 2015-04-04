#include <boost/thread/lock_guard.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include "gpuimageproc/GPUConfig.h"
#include "gpuimageproc/connectedtopics.h"

namespace gpuimageproc
{
class Stereoproc : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_l_raw_image_, sub_r_raw_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info_, sub_r_info_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
          sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
          sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_mono_left_;
  ros::Publisher pub_mono_right_;
  ros::Publisher pub_color_left_;
  ros::Publisher pub_color_right_;
  ros::Publisher pub_mono_rect_left_;
  ros::Publisher pub_mono_rect_right_;
  ros::Publisher pub_color_rect_left_;
  ros::Publisher pub_color_rect_right_;
  ros::Publisher pub_disparity_;
  ros::Publisher pub_disparity_vis_;
  ros::Publisher pub_pointcloud_;
  struct ConnectedTopics connected_;

  stereo_msgs::DisparityImagePtr disp_msg_;
  cv::Mat_<float> disp_msg_data_;
  cv::gpu::CudaMem filter_buf_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef gpuimageproc::GPUConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;

  cv::gpu::StereoBM_GPU block_matcher_;
  const int block_matcher_min_disparity_ = 0;

  cv::gpu::StereoConstantSpaceBP csbp_;

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& l_raw_msg, const sensor_msgs::CameraInfoConstPtr& l_info_msg,
               const sensor_msgs::ImageConstPtr& r_raw_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  void configCb(Config &config, uint32_t level);

  void sendDisparity(void);

  void filterSpeckles(void);
  int maxSpeckleSize_;
  double maxDiff_;

public:
  virtual void onInit();
};

}
