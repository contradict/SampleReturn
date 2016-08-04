#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <samplereturn_msgs/PatchArray.h>
#include <samplereturn_msgs/Patch.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/ground_projector_paramsConfig.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

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

class GroundProjectorNode
{
  ros::Subscriber sub_patch_array;
  ros::Subscriber sub_point_cloud;
  ros::Publisher pub_point_cloud;
  ros::Publisher pub_patch_array;
  ros::Publisher pub_marker;
  ros::Publisher pub_point;
  ros::Publisher pub_debug_image;
  std::string sub_patch_array_topic;
  std::string pub_patch_array_topic;
  std::string sub_point_cloud_topic;
  std::string pub_point_cloud_topic;
  std::string pub_debug_image_topic;
  std::string marker_topic;
  std::string point_topic;

  tf::TransformListener listener_;

  dynamic_reconfigure::Server<saliency_detector::ground_projector_paramsConfig> dr_srv;

  double ransac_distance_threshold_;
  int min_inliers_;
  double max_height_;
  double min_major_axis_, max_major_axis_;
  bool enable_debug_;

  Eigen::Vector4d ground_plane_;
  image_geometry::PinholeCameraModel cam_model_;

  cv::Mat debug_image_;

  public:
  GroundProjectorNode() {
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<saliency_detector::ground_projector_paramsConfig>::CallbackType cb;
    cb = boost::bind(&GroundProjectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("sub_patch_array_topic", sub_patch_array_topic,
        std::string("patch_array"));
    private_node_handle_.param("pub_patch_array_topic", pub_patch_array_topic,
        std::string("projected_patch_array"));
    private_node_handle_.param("sub_point_cloud_topic", sub_point_cloud_topic,
        std::string("points2"));
    private_node_handle_.param("pub_point_cloud_topic", pub_point_cloud_topic,
        std::string("ground_pointcloud"));
    private_node_handle_.param("marker_topic", marker_topic,
        std::string("ground_marker"));
    private_node_handle_.param("point_topic", point_topic,
        std::string("ground_point"));
    private_node_handle_.param("pub_debug_image_topic", pub_debug_image_topic,
        std::string("projector_debug_image"));

    sub_patch_array =
      nh.subscribe(sub_patch_array_topic, 1, &GroundProjectorNode::patchArrayCallback, this);

    sub_point_cloud =
      nh.subscribe(sub_point_cloud_topic, 1, &GroundProjectorNode::pointCloudCallback, this);

    pub_point_cloud =
      nh.advertise<sensor_msgs::PointCloud2>(pub_point_cloud_topic.c_str(), 1);

    pub_patch_array =
      nh.advertise<samplereturn_msgs::PatchArray>(pub_patch_array_topic.c_str(), 1);

    pub_marker =
      nh.advertise<visualization_msgs::Marker>(marker_topic.c_str(), 1);

    pub_point =
      nh.advertise<geometry_msgs::PointStamped>(point_topic.c_str(), 1);

    pub_debug_image =
      nh.advertise<sensor_msgs::Image>(pub_debug_image_topic.c_str(), 1);

    min_inliers_ = 30;
    ransac_distance_threshold_ = 1.0;
    max_height_ = 1.0;
    min_major_axis_ = 0.04;
    max_major_axis_ = 0.12;
    enable_debug_ = false;
  }

  // For a set of candidate sample patches, project them onto the ground to get
  // 3D location and size estimate. If stereo data is available, usse that estimated
  // ground plane, otherwise assume the plane under the wheels.
  void patchArrayCallback(const samplereturn_msgs::PatchArrayConstPtr& msg)
  {
    if (msg->patch_array.empty()) {
      return;
    }
    samplereturn_msgs::PatchArray out_pa_msg;
    if (enable_debug_) {
      debug_image_ = cv::Mat::ones(msg->patch_array[0].cam_info.height,
          msg->patch_array[0].cam_info.width, CV_8U)*255;
    }
    if (!cam_model_.initialized())
    {
      cam_model_.fromCameraInfo(msg->patch_array[0].cam_info);
    }
    // For each patch, project onto the available ground plane and make
    // accurate 3D position and size estimates. Filter out candidate patches
    // that are too large or small.
    // Take the saliency contour, find the major axis, cast rays to
    // the ground plane
    cv_bridge::CvImagePtr cv_ptr;
    for (int i = 0; i < msg->patch_array.size(); i++) {
      try {
        cv_ptr = cv_bridge::toCvCopy(msg->patch_array[i].mask, "");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
      // Get largest contour
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      // findContours is destructive, hand in image copy
      cv::Mat contour_copy = cv_ptr->image.clone();
      cv::findContours(contour_copy, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
      if (contours.size() == 0) {
        ROS_DEBUG("No contours found in patch %i", i);
      }
      double maxArea = 0;
      int max_idx = 0;
      for (int j=0; j<contours.size(); j++) {
        double area = cv::contourArea(cv::Mat(contours[j]));
        if (area > maxArea) {
          maxArea = area;
          max_idx = j;
        }
      }
      // Find min enclosing rectangle, major axis
      cv::RotatedRect rect = cv::minAreaRect(contours[max_idx]);
      if (enable_debug_) {
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int k = 0; k < 4; k++) {
          cv::line(cv_ptr->image, vertices[k], vertices[(k+1)%4], 255, 10);
        }
        cv_ptr->image.copyTo(debug_image_(cv::Rect(msg->patch_array[i].image_roi.x_offset,
                msg->patch_array[i].image_roi.y_offset,
                msg->patch_array[i].image_roi.width,
                msg->patch_array[i].image_roi.height)));
      }
      // Project major axis end rays to plane, compute major axis in meters
      cv::Point2d roi_offset(msg->patch_array[i].image_roi.x_offset,
          msg->patch_array[i].image_roi.y_offset);
      cv::Point2d major_point_a, major_point_b;
      cv::Point2d rect_major_point_a, rect_major_point_b;
      getMajorPointsFullImage(rect, roi_offset, major_point_a, major_point_b);
      ROS_DEBUG("Major Point A: %f, %f",major_point_a.x,major_point_a.y);
      ROS_DEBUG("Major Point B: %f, %f",major_point_b.x,major_point_b.y);
      rect_major_point_a = cam_model_.rectifyPoint(major_point_a);
      rect_major_point_b = cam_model_.rectifyPoint(major_point_b);
      ROS_DEBUG("Rect Major Point A: %f, %f",rect_major_point_a.x,rect_major_point_a.y);
      ROS_DEBUG("Rect Major Point B: %f, %f",rect_major_point_b.x,rect_major_point_b.y);
      cv::Point3d ray_a = cam_model_.projectPixelTo3dRay(rect_major_point_a);
      cv::Point3d ray_b = cam_model_.projectPixelTo3dRay(rect_major_point_b);
      ROS_DEBUG("Ray A: %f, %f, %f",ray_a.x,ray_a.y,ray_a.z);
      ROS_DEBUG("Ray B: %f, %f, %f",ray_b.x,ray_b.y,ray_b.z);
      geometry_msgs::PointStamped world_point;
      if (checkContourSize(ray_a, ray_b, msg->patch_array[0].header, world_point)) {
        pub_point.publish(world_point);
        samplereturn_msgs::Patch pa_msg;
        pa_msg.image = msg->patch_array[i].image;
        pa_msg.mask = msg->patch_array[i].mask;
        pa_msg.image_roi = msg->patch_array[i].image_roi;
        pa_msg.header = msg->patch_array[i].header;
        pa_msg.cam_info = msg->patch_array[i].cam_info;
        pa_msg.world_point = world_point;
        out_pa_msg.patch_array.push_back(pa_msg);
        pub_patch_array.publish(out_pa_msg);
      }
      else {
        if (enable_debug_) {
          cv::line(debug_image_,
              cv::Point2f(msg->patch_array[i].image_roi.x_offset,
                          msg->patch_array[i].image_roi.y_offset),
              cv::Point2f(msg->patch_array[i].image_roi.x_offset +
                          msg->patch_array[i].image_roi.width,
                          msg->patch_array[i].image_roi.y_offset +
                          msg->patch_array[i].image_roi.height), 255, 50);
          cv::line(debug_image_,
              cv::Point2f(msg->patch_array[i].image_roi.x_offset +
                          msg->patch_array[i].image_roi.width,
                          msg->patch_array[i].image_roi.y_offset),
              cv::Point2f(msg->patch_array[i].image_roi.x_offset,
                          msg->patch_array[i].image_roi.y_offset +
                          msg->patch_array[i].image_roi.height), 255, 50);
        }
        continue;
      }
    }
    if (enable_debug_) {
      sensor_msgs::ImagePtr debug_image_msg =
        cv_bridge::CvImage(msg->patch_array[0].header,"mono8",debug_image_).toImageMsg();
      pub_debug_image.publish(debug_image_msg);
    }
  }

  void getMajorPointsFullImage(const cv::RotatedRect rect, cv::Point2d roi_offset,
      cv::Point2d& major_point_a, cv::Point2d& major_point_b)
  {
    major_point_a.x = rect.center.x - (rect.size.width/2.)*cos(rect.angle*(M_PI/180.))
                      + roi_offset.x;
    major_point_a.y = rect.center.y - (rect.size.height/2.)*sin(rect.angle*(M_PI/180.))
                      + roi_offset.y;
    major_point_b.x = rect.center.x + (rect.size.width/2.)*cos(rect.angle*(M_PI/180.))
                      + roi_offset.x;
    major_point_b.y = rect.center.y + (rect.size.height/2.)*sin(rect.angle*(M_PI/180.))
                      + roi_offset.y;
    return;
  }

  bool checkContourSize(const cv::Point3d ray_a, const cv::Point3d ray_b,
      const std_msgs::Header header, geometry_msgs::PointStamped & ground_point)
  {
    // Take each ray in their origin frame, intersect with estimated ground plane
    geometry_msgs::PointStamped camera_point_a, base_link_point_a;
    geometry_msgs::PointStamped camera_point_b, base_link_point_b;
    camera_point_a.header = header;
    camera_point_a.point.x = ray_a.x;
    camera_point_a.point.y = ray_a.y;
    camera_point_a.point.z = ray_a.z;
    camera_point_b.header = header;
    camera_point_b.point.x = ray_b.x;
    camera_point_b.point.y = ray_b.y;
    camera_point_b.point.z = ray_b.z;
    tf::StampedTransform camera_transform;
    // This is a static link, so Time(0) should be fine
    listener_.lookupTransform("base_link",header.frame_id,ros::Time(0),camera_transform);
    if (!listener_.canTransform("base_link",camera_point_a.header.frame_id,
          camera_point_a.header.stamp)) {
      ROS_INFO("Couldn't transform base_link to %s\n",camera_point_a.header.frame_id.c_str());
      return false;
    }
    listener_.transformPoint("base_link",camera_point_a,base_link_point_a);
    listener_.transformPoint("base_link",camera_point_b,base_link_point_b);
    Eigen::Vector3d base_link_ray_a, base_link_ray_b, ray_origin;
    tf::Vector3 pos;
    pos = camera_transform.getOrigin();
    ROS_DEBUG("Camera Pos in Base Link: %f, %f, %f",pos.x(),pos.y(),pos.z());
    ray_origin[0] = pos.x();
    ray_origin[1] = pos.y();
    ray_origin[2] = pos.z();
    base_link_ray_a[0] = base_link_point_a.point.x - pos.x();
    base_link_ray_a[1] = base_link_point_a.point.y - pos.y();
    base_link_ray_a[2] = base_link_point_a.point.z - pos.z();
    base_link_ray_b[0] = base_link_point_b.point.x - pos.x();
    base_link_ray_b[1] = base_link_point_b.point.y - pos.y();
    base_link_ray_b[2] = base_link_point_b.point.z - pos.z();
    Eigen::Vector3d ground_point_a = intersectRayPlane(base_link_ray_a, ray_origin, ground_plane_);
    Eigen::Vector3d ground_point_b = intersectRayPlane(base_link_ray_b, ray_origin, ground_plane_);
    ROS_DEBUG("Ground Point A: %f, %f, %f",ground_point_a.x(),ground_point_a.y(),ground_point_a.z());
    ROS_DEBUG("Ground Point B: %f, %f, %f",ground_point_b.x(),ground_point_b.y(),ground_point_b.z());
    Eigen::Vector3d mid_ground_point = (ground_point_a + ground_point_b)/2;
    ROS_DEBUG("Ground Point Mid: %f, %f, %f",mid_ground_point.x(),mid_ground_point.y(),mid_ground_point.z());
    if (((ground_point_b - ground_point_a).norm() < max_major_axis_) &&
        ((ground_point_b - ground_point_a).norm() > min_major_axis_)) {
      ground_point.header.stamp = header.stamp;
      ground_point.header.frame_id = "base_link";
      ground_point.point.x = mid_ground_point[0];
      ground_point.point.y = mid_ground_point[1];
      ground_point.point.z = mid_ground_point[2];
      listener_.transformPoint("odom",ground_point,ground_point);
      return true;
    }
    else {
      return false;
    }
  }

  Eigen::Vector3d intersectRayPlane(const Eigen::Vector3d ray,
      const Eigen::Vector3d ray_origin, const Eigen::Vector4d plane)
  {
    // Plane defined as ax + by + cz + d = 0
    // Ray: P = P0 + tV
    // Plane: P.N + d = 0, where P is intersection point
    // t = -(P0.N + d)/(V.N) , P = P0 + tV
    float t = -(ray_origin.dot(plane.head(3)) + plane[3]) / (ray.dot(plane.head(3)));
    Eigen::Vector3d P = ray_origin + t*ray;
    return P;
  }

  // Take the associated stereo point cloud, trim and extract a ground plane
  // for projecting candidate patches onto.
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Transform points to base_link frame
    sensor_msgs::PointCloud2 base_link_points_msg;
    pcl_ros::transformPointCloud("/base_link",*msg,base_link_points_msg,listener_);

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(base_link_points_msg, pcl_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc,*ptr_cloud);

    // Clip points to below some height above base_link z=0
    Eigen::Vector4d new_plane;
    new_plane.setZero();
    new_plane[2] = -1.0;
    new_plane[3] = max_height_;
    pcl::PlaneClipper3D<pcl::PointXYZ> ground_clipper(new_plane.cast<float>());
    pcl::PointIndices::Ptr clipped (new pcl::PointIndices);
    std::vector<int> indices;
    ground_clipper.clipPointCloud3D(*ptr_cloud,clipped->indices,indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (ptr_cloud);
    extract.setIndices (clipped);
    extract.setNegative (false);
    extract.filter (*clipped_cloud);

    pcl::toPCLPointCloud2(*clipped_cloud,pcl_pc);
    pcl_conversions::fromPCL(pcl_pc,base_link_points_msg);

    base_link_points_msg.header.stamp = msg->header.stamp;
    base_link_points_msg.header.frame_id = "/base_link";
    pub_point_cloud.publish(base_link_points_msg);

    // Fit ground plane. This doesn't allow for arbitrarily complex ground geometry,
    // but will handle local deviations in robot attitude and hillside approaches
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (false);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold (ransac_distance_threshold_);

    // Check if cloud is empty //
    if (clipped_cloud->points.size() <= min_inliers_) {
      ROS_DEBUG("Could not estimate a planar model for given dataset, not\
          enough points at input.");
      return;
    }
    seg.setInputCloud(clipped_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() <= min_inliers_) {
      ROS_DEBUG("Could not estimate a planar model for given dataset, not\
          enough inliers");
      return;
    }
    // Keep the plane around for patch projection, publish the clipped ground point cloud,
    // and a plane marker for Rviz
    ROS_DEBUG("Model Coefficients: %f, %f, %f, %f",coefficients->values[0],
                                                  coefficients->values[1],
                                                  coefficients->values[2],
                                                  coefficients->values[3]);

    visualization_msgs::Marker mark;
    mark.header = msg->header;
    mark.header.frame_id = "/base_link";
    Eigen::Quaterniond orientation;
    Eigen::Vector3d xhat;
    xhat.setZero();
    xhat[0] = 1.0;
    ground_plane_ << coefficients->values[0],
                  coefficients->values[1],
                  coefficients->values[2],
                  coefficients->values[3];
    orientation.setFromTwoVectors(xhat, ground_plane_.head(3));
    tf::quaternionEigenToMsg(orientation, mark.pose.orientation);
    mark.scale.x = 1.0;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    mark.color.a = 1.0;
    mark.color.r = 0.0;
    mark.color.g = 0.5;
    mark.color.b = 1.0;
    pub_marker.publish(mark);
  }

  void configCallback(saliency_detector::ground_projector_paramsConfig &config, uint32_t level)
  {
    ransac_distance_threshold_ = config.ransac_distance_threshold;
    min_inliers_ = config.min_inliers;
    max_height_ = config.max_height;
    min_major_axis_ = config.min_major_axis;
    max_major_axis_ = config.max_major_axis;
    enable_debug_ = config.enable_debug;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_projector");
  GroundProjectorNode gp;
  ros::spin();
}
