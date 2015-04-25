#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <fence_detection/fence_detector_paramsConfig.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <eigen_conversions/eigen_msg.h>

class FenceDetectorNode
{
  ros::NodeHandle nh;
  ros::Subscriber color_img_sub;
  ros::Subscriber disparity_img_sub;
  ros::Subscriber l_camera_info_sub;
  ros::Subscriber r_camera_info_sub;
  ros::Publisher mask_pub;
  ros::Publisher points_pub;
  ros::Publisher marker_pub;
  ros::Publisher line_marker_pub;
  ros::Publisher fence_line_pub;

  std::string color_img_topic = "color_image";
  std::string disparity_img_topic = "disparity_image";
  std::string l_camera_info_topic = "left/camera_info";
  std::string r_camera_info_topic = "right/camera_info";
  std::string mask_pub_topic = "fence_mask";
  std::string points_pub_topic = "points";
  std::string marker_pub_topic = "plane_marker";
  std::string line_marker_pub_topic = "line_marker";
  std::string fence_line_pub_topic = "fence_line";

  double a_channel_threshold_, b_channel_threshold_, x_edge_threshold_, y_edge_threshold_, sum_threshold_;
  double max_range_, min_height_;
  int dilate_iterations_, erode_iterations_;
  double ransac_distance_threshold_;
  int min_inliers_;

  sensor_msgs::CameraInfo r_cam_info_;
  image_geometry::StereoCameraModel cam_model_;
  cv::Mat_<cv::Vec3f> points_mat_;
  stereo_msgs::DisparityImage disp_msg_;

  dynamic_reconfigure::Server<fence_detection::fence_detector_paramsConfig> dr_srv;

  tf::TransformListener listener_;

  public:
  FenceDetectorNode() {

    dynamic_reconfigure::Server<fence_detection::fence_detector_paramsConfig>::CallbackType cb;

    cb = boost::bind(&FenceDetectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("a_channel_threshold",a_channel_threshold_,double(170));
    private_node_handle_.param("b_channel_threshold",b_channel_threshold_,double(170));
    private_node_handle_.param("x_edge_threshold",x_edge_threshold_,double(150));
    private_node_handle_.param("y_edge_threshold",y_edge_threshold_,double(150));
    private_node_handle_.param("sum_threshold",sum_threshold_,double(85));
    private_node_handle_.param("dilate_iterations",dilate_iterations_,int(14));
    private_node_handle_.param("erode_iterations",erode_iterations_,int(20));
    private_node_handle_.param("ransac_distance_threshold",ransac_distance_threshold_,double(0.1));

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

    marker_pub =
      nh.advertise<visualization_msgs::Marker>(marker_pub_topic.c_str(), 3);

    line_marker_pub =
      nh.advertise<visualization_msgs::Marker>(line_marker_pub_topic.c_str(), 3);

    fence_line_pub =
      nh.advertise<geometry_msgs::PolygonStamped>(fence_line_pub_topic.c_str(), 3);

  }

  void l_cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    cam_model_.fromCameraInfo(msg,r_cam_info_);
  }

  void r_cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    r_cam_info_ = msg;
  }

  void disparityCallback(const stereo_msgs::DisparityImage& msg)
  {
    cv::Mat_<float> dmat(msg.image.height, msg.image.width,
        (float*)&msg.image.data[0], msg.image.step);
    cam_model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  }

  void imageCallback(const sensor_msgs::Image& msg)
  {
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
    cv::Mat b(lab.rows,lab.cols,CV_8UC1);
    cv::Mat x_edge, y_edge;
    cv::Mat x_thresh, y_thresh, a_thresh, b_thresh;
    cv::Mat x_mask, y_mask, a_mask, b_mask, sum_mask;
    int from_to[] = { 1,0 , 2,0};
    cv::Mat out[] = {a,b};
    cv::mixChannels( &lab, 1, out, 1, from_to, 1);
    cv::Sobel(a, x_edge, CV_32F, 1, 0, 3, 1, 128);
    cv::Sobel(a, y_edge, CV_32F, 0, 1, 3, 1, 128);
    x_edge.convertTo(x_edge,CV_8U);
    y_edge.convertTo(y_edge,CV_8U);
    cv::threshold(x_edge, x_thresh, x_edge_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(y_edge, y_thresh, y_edge_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(a, a_thresh, a_channel_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(b, b_thresh, b_channel_threshold_, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::dilate(x_thresh, x_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::dilate(y_thresh, y_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::dilate(a_thresh, a_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::dilate(b_thresh, b_mask, kernel, cv::Point(-1,-1), dilate_iterations_);
    cv::erode(x_mask, x_mask, kernel, cv::Point(-1,-1), erode_iterations_);
    cv::erode(y_mask, y_mask, kernel, cv::Point(-1,-1), erode_iterations_);
    cv::erode(a_mask, a_mask, kernel, cv::Point(-1,-1), erode_iterations_);
    cv::erode(b_mask, b_mask, kernel, cv::Point(-1,-1), erode_iterations_);

    x_mask.convertTo(x_mask,CV_32F);
    y_mask.convertTo(y_mask,CV_32F);
    a_mask.convertTo(a_mask,CV_32F);
    b_mask.convertTo(b_mask,CV_32F);

    cv::threshold((x_mask+y_mask+a_mask+b_mask)/float(4), sum_mask, sum_threshold_, 255,
        cv::THRESH_BINARY);

    sum_mask.convertTo(sum_mask,CV_8U);

    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"mono8",sum_mask).toImageMsg();
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

    /* Transform points to base_link frame */
    sensor_msgs::PointCloud2 base_link_points_msg;
    pcl_ros::transformPointCloud("/base_link",*points_msg,base_link_points_msg,listener_);

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(base_link_points_msg, pcl_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far_clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc,*ptr_cloud);

    /* Trim ground points */
    Eigen::Vector4f ground_plane;
    ground_plane.setZero();
    ground_plane[2] = 1.0;
    ground_plane[3] = -min_height_;
    pcl::PlaneClipper3D<pcl::PointXYZ> ground_clipper(ground_plane);
    //pcl::PointIndices::Ptr clipped (new pcl::PointIndices);
    //std::vector<int> clipped;
    pcl::PointIndices::Ptr clipped (new pcl::PointIndices);
    std::vector<int> indices;
    ground_clipper.clipPointCloud3D(*ptr_cloud,clipped->indices,indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (ptr_cloud);
    extract.setIndices (clipped);
    extract.setNegative (false);
    extract.filter (*clipped_cloud);

    /*Trim far points */
    Eigen::Vector4f far_plane;
    far_plane.setZero();
    far_plane[0] = -1.0;
    far_plane[3] = max_range_;
    pcl::PlaneClipper3D<pcl::PointXYZ> far_clipper(far_plane);
    pcl::PointIndices::Ptr far_clipped (new pcl::PointIndices);
    std::vector<int> far_indices;
    far_clipper.clipPointCloud3D(*clipped_cloud,far_clipped->indices,far_indices);
    pcl::ExtractIndices<pcl::PointXYZ> far_extract;
    far_extract.setInputCloud (clipped_cloud);
    far_extract.setIndices (far_clipped);
    far_extract.setNegative (false);
    far_extract.filter (*far_clipped_cloud);

    pcl::toPCLPointCloud2(*far_clipped_cloud,pcl_pc);
    pcl_conversions::fromPCL(pcl_pc,base_link_points_msg);
    points_pub.publish(base_link_points_msg);

    /* Time to fit the points */
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold (ransac_distance_threshold_);

    ROS_DEBUG("Width, Height:%u, %u",ptr_cloud->width,ptr_cloud->height);

    seg.setInputCloud(far_clipped_cloud);
    //seg.setInputCloud(ptr_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() <= min_inliers_) {
      ROS_ERROR("Could not estimate a planar model for given dataset.");
      return;
    }
    ROS_DEBUG("Model Coefficients: %f, %f, %f, %f",coefficients->values[0],
                                                  coefficients->values[1],
                                                  coefficients->values[2],
                                                  coefficients->values[3]);
    Eigen::Vector3d centroid;
    centroid.setZero();
    //for(size_t i=0; i<inliers->indices.size(); i++) {
    //  centroid += inliers->points[
    visualization_msgs::Marker mark;
    mark.header = msg.header;
    mark.header.frame_id = "/base_link";
    Eigen::Quaterniond orientation;
    Eigen::Vector3d xhat;
    xhat.setZero();
    xhat[0] = 1.0;
    Eigen::Vector3d plane_normal;
    plane_normal << coefficients->values[0],
                 coefficients->values[1],
                 coefficients->values[2];
    orientation.setFromTwoVectors(xhat, plane_normal);
    tf::quaternionEigenToMsg(orientation, mark.pose.orientation);
    mark.scale.x = 1.0;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    mark.color.a = 1.0;
    mark.color.r = 0.0;
    mark.color.g = 0.5;
    mark.color.b = 1.0;
    marker_pub.publish(mark);

    visualization_msgs::Marker line_mark;
    line_mark.header = msg.header;
    line_mark.header.frame_id = "/base_link";
    line_mark.color.a = 1.0;
    line_mark.color.r = 1.0;
    line_mark.color.g = 0.4;
    line_mark.color.b = 0.0;
    line_mark.scale.x = 0.4;
    geometry_msgs::Point p;
    line_mark.pose.position.x = 2.0* -coefficients->values[3]/coefficients->values[0];
    p.z = 0.0;
    p.y = -5.0;
    p.x = (coefficients->values[3] - coefficients->values[1]*p.y)/coefficients->values[0];
    line_mark.points.push_back(p);
    p.y = 5.0;
    p.x = (coefficients->values[3] - coefficients->values[1]*p.y)/coefficients->values[0];
    line_mark.points.push_back(p);
    line_marker_pub.publish(line_mark);

    geometry_msgs::PolygonStamped fence_line;
    fence_line.header = msg.header;
    fence_line.header.frame_id = "/base_link";
    geometry_msgs::Point32 p1;
    p1.y = -5.0;
    p1.x = (coefficients->values[3] - coefficients->values[1]*p1.y)/coefficients->values[0];
    p1.z = 0.0;
    fence_line.polygon.points.push_back(p1);
    geometry_msgs::Point32 p2;
    p2.y = -5.0;
    p2.x = (coefficients->values[3] - coefficients->values[1]*p2.y)/coefficients->values[0];
    p2.z = 0.0;
    fence_line.polygon.points.push_back(p2);
    fence_line_pub.publish(fence_line);
  }

  /* Dynamic reconfigure callback */
  void configCallback(fence_detection::fence_detector_paramsConfig &config, uint32_t level)
  {
    ROS_DEBUG("configCallback");
    x_edge_threshold_ = config.x_edge_threshold;
    y_edge_threshold_ = config.y_edge_threshold;
    a_channel_threshold_ = config.a_channel_threshold;
    b_channel_threshold_ = config.b_channel_threshold;
    sum_threshold_ = config.sum_threshold;
    dilate_iterations_ = config.dilate_iterations;
    erode_iterations_ = config.erode_iterations;
    min_height_ = config.min_height;
    max_range_ = config.max_range;
    ransac_distance_threshold_ = config.ransac_distance_threshold;
    min_inliers_ = config.min_inliers;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fence_detector");
  FenceDetectorNode fd;
  ros::spin();
}
