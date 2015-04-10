#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <dynamic_reconfigure/server.h>
#include <fence_detection/fence_detector_paramsConfig.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

class FenceDetectorNode
{
  ros::NodeHandle nh;
  ros::Subscriber color_img_sub;
  ros::Subscriber disparity_img_sub;
  ros::Subscriber l_camera_info_sub;
  ros::Subscriber r_camera_info_sub;
  ros::Publisher mask_pub;
  ros::Publisher points_pub;

  std::string color_img_topic = "color_image";
  std::string disparity_img_topic = "disparity_image";
  std::string l_camera_info_topic = "left/camera_info";
  std::string r_camera_info_topic = "right/camera_info";
  std::string mask_pub_topic = "fence_mask";
  std::string points_pub_topic = "points";

  double a_channel_threshold_, x_edge_threshold_, y_edge_threshold_, sum_threshold_;
  int dilate_iterations_, erode_iterations_;

  sensor_msgs::CameraInfo r_cam_info_;
  image_geometry::StereoCameraModel cam_model_;
  cv::Mat_<cv::Vec3f> points_mat_;
  stereo_msgs::DisparityImage disp_msg_;

  dynamic_reconfigure::Server<fence_detection::fence_detector_paramsConfig> dr_srv;

  public:
  FenceDetectorNode() {

    dynamic_reconfigure::Server<fence_detection::fence_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&FenceDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("a_channel_threshold",a_channel_threshold_,double(170));
    private_node_handle_.param("x_edge_threshold",x_edge_threshold_,double(150));
    private_node_handle_.param("y_edge_threshold",y_edge_threshold_,double(150));
    private_node_handle_.param("sum_threshold",sum_threshold_,double(85));
    private_node_handle_.param("dilate_iterations",dilate_iterations_,int(14));
    private_node_handle_.param("erode_iterations",erode_iterations_,int(20));

    color_img_sub =
      nh.subscribe(color_img_topic.c_str(), 3, &FenceDetectorNode::imageCallback, this);

    disparity_img_sub =
      nh.subscribe(disparity_img_topic.c_str(), 3, &FenceDetectorNode::disparityCallback, this);

    l_camera_info_sub =
      nh.subscribe(l_camera_info_topic.c_str(), 3, &FenceDetectorNode::l_cameraInfoCallback, this);

    r_camera_info_sub =
      nh.subscribe(r_camera_info_topic.c_str(), 3, &FenceDetectorNode::r_cameraInfoCallback, this);

    mask_pub =
      nh.advertise<sensor_msgs::Image>(mask_pub_topic.c_str(), 3);

    points_pub =
      nh.advertise<sensor_msgs::PointCloud2>(points_pub_topic.c_str(), 3);

  }

  void l_cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    ROS_INFO("Left Camera Info Callback");
    cam_model_.fromCameraInfo(msg,r_cam_info_);
  }

  void r_cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    ROS_INFO("Right Camera Info Callback");
    r_cam_info_ = msg;
  }

  void disparityCallback(const stereo_msgs::DisparityImage& msg)
  {
    ROS_INFO("Disparity Callback");
    cv::Mat_<float> dmat(msg.image.height, msg.image.width,
        (float*)&msg.image.data[0], msg.image.step);
    cam_model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  }

  void imageCallback(const sensor_msgs::Image& msg)
  {
    ROS_INFO("Image Callback");
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "");
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat lab;
    cv::cvtColor(cv_ptr->image, lab, CV_BGR2Lab);
    cv::Mat a(lab.rows,lab.cols,CV_8UC1);
    cv::Mat x_edge, y_edge;
    cv::Mat x_thresh, y_thresh, a_thresh;
    cv::Mat x_mask, y_mask, a_mask, sum_mask;
    int from_to[] = { 1,0 };
    cv::mixChannels( &lab, 1, &a, 1, from_to, 1);
    cv::Sobel(a, x_edge, CV_32F, 1, 0, 3, 1, 128);
    cv::Sobel(a, y_edge, CV_32F, 0, 1, 3, 1, 128);
    x_edge.convertTo(x_edge,CV_8U);
    y_edge.convertTo(y_edge,CV_8U);
    cv::threshold(x_edge, x_thresh, x_edge_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(y_edge, y_thresh, y_edge_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(a, a_thresh, a_channel_threshold_, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::dilate(x_thresh, x_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::dilate(y_thresh, y_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::dilate(a_thresh, a_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::erode(x_mask, x_mask, kernel, cv::Point(-1,-1), erode_iterations_);
    cv::erode(y_mask, y_mask, kernel, cv::Point(-1,-1), erode_iterations_);
    cv::erode(a_mask, a_mask, kernel, cv::Point(-1,-1), erode_iterations_);

    x_mask.convertTo(x_mask,CV_32F);
    y_mask.convertTo(y_mask,CV_32F);
    a_mask.convertTo(a_mask,CV_32F);

    cv::threshold((x_mask+y_mask+a_mask)/float(3), sum_mask, sum_threshold_, 255,
        cv::THRESH_BINARY);

    sum_mask.convertTo(sum_mask,CV_8U);

    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"mono8",x_thresh).toImageMsg();
    mask_pub.publish(debug_img_msg);

    sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    points_msg->header = msg.header;
    points_msg->height = points_mat_.rows;
    points_msg->width = points_mat_.cols;
    points_msg->is_bigendian = false;
    points_msg->is_dense = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");

    float bad_point = std::numeric_limits<float>::quiet_NaN();
    for (int v = 0; v<points_mat_.rows; ++v)
    {
      for (int u=0; u<points_mat_.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
      {
        if (points_mat_(v,u)[2]!=image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(points_mat_(v,u)[2]) && sum_mask.at<uchar>(v,u)!=0)
        {
          *iter_x = points_mat_(v,u)[0];
          *iter_y = points_mat_(v,u)[1];
          *iter_z = points_mat_(v,u)[2];
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
        }
      }
    }
    points_pub.publish(points_msg);
  }

  /* Dynamic reconfigure callback */
  void configCallback(fence_detection::fence_detector_paramsConfig &config, uint32_t level)
  {
    ROS_INFO("configCallback");
    x_edge_threshold_ = config.x_edge_threshold;
    y_edge_threshold_ = config.y_edge_threshold;
    a_channel_threshold_ = config.a_channel_threshold;
    sum_threshold_ = config.sum_threshold;
    dilate_iterations_ = config.dilate_iterations;
    erode_iterations_ = config.erode_iterations;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fence_detector");
  FenceDetectorNode fd;
  ros::spin();
}
