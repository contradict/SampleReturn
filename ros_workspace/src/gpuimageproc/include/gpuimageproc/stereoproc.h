#include <boost/thread/lock_guard.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <gpuimageproc/GPUConfig.h>
#include <dynamic_reconfigure/server.h>

#include <gpuimageproc/connectedtopics.h>

namespace gpuimageproc
{
class Stereoproc : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info_, sub_r_info_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_mono_;
  ros::Publisher pub_color_;
  ros::Publisher pub_mono_rect_;
  ros::Publisher pub_color_rect_;
  ros::Publisher pub_disparity_;
  ros::Publisher pub_pointcloud_;
  struct ConnectedTopics connected;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef gpuimageproc::GPUConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::CameraInfoConstPtr& l_info_msg,
               const sensor_msgs::ImageConstPtr& r_image_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  void configCb(Config &config, uint32_t level);
};

}
