#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <samplereturn_msgs/PatchArray.h>
#include <samplereturn_msgs/Patch.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/ground_projector_paramsConfig.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <eigen_conversions/eigen_msg.h>

class GroundProjectorNode
{
  ros::Subscriber sub_patch_array;
  ros::Subscriber sub_plane_model;
  ros::Publisher pub_patch_array;
  ros::Publisher pub_marker;
  ros::Publisher pub_point;
  ros::Publisher pub_debug_image;
  std::string sub_patch_array_topic;
  std::string pub_patch_array_topic;
  std::string sub_plane_model_topic;
  std::string pub_debug_image_topic;
  std::string marker_topic;
  std::string point_topic;

  tf::TransformListener listener_;

  dynamic_reconfigure::Server<saliency_detector::ground_projector_paramsConfig> dr_srv;

  double min_major_axis_, max_major_axis_;
  bool enable_debug_;
  int miss_count_;

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
    private_node_handle_.param("sub_plane_model_topic", sub_plane_model_topic,
        std::string("model"));
    private_node_handle_.param("marker_topic", marker_topic,
        std::string("ground_marker"));
    private_node_handle_.param("point_topic", point_topic,
        std::string("ground_point"));
    private_node_handle_.param("pub_debug_image_topic", pub_debug_image_topic,
        std::string("projector_debug_image"));

    sub_patch_array =
      nh.subscribe(sub_patch_array_topic, 1, &GroundProjectorNode::patchArrayCallback, this);

    sub_plane_model =
      nh.subscribe(sub_plane_model_topic, 1, &GroundProjectorNode::planeModelCallback, this);

    pub_patch_array =
      nh.advertise<samplereturn_msgs::PatchArray>(pub_patch_array_topic.c_str(), 1);

    pub_marker =
      nh.advertise<visualization_msgs::Marker>(marker_topic.c_str(), 1);

    pub_point =
      nh.advertise<geometry_msgs::PointStamped>(point_topic.c_str(), 1);

    pub_debug_image =
      nh.advertise<sensor_msgs::Image>(pub_debug_image_topic.c_str(), 1);

    enable_debug_ = false;
    ground_plane_ << 0.,0.,1.,0.;
    miss_count_ = 0;
  }

  // For a set of candidate sample patches, project them onto the ground to get
  // 3D location and size estimate. If stereo data is available, usse that estimated
  // ground plane, otherwise assume the plane under the wheels.
  void patchArrayCallback(const samplereturn_msgs::PatchArrayConstPtr& msg)
  {
    enable_debug_ = (pub_debug_image.getNumSubscribers() != 0);
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
        continue;
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
      }
      else {
        if (enable_debug_) {
          cv::line(debug_image_,
              cv::Point2f(msg->patch_array[i].image_roi.x_offset,
                          msg->patch_array[i].image_roi.y_offset),
              cv::Point2f(msg->patch_array[i].image_roi.x_offset +
                          msg->patch_array[i].image_roi.width,
                          msg->patch_array[i].image_roi.y_offset +
                          msg->patch_array[i].image_roi.height), 255, 20);
          cv::line(debug_image_,
              cv::Point2f(msg->patch_array[i].image_roi.x_offset +
                          msg->patch_array[i].image_roi.width,
                          msg->patch_array[i].image_roi.y_offset),
              cv::Point2f(msg->patch_array[i].image_roi.x_offset,
                          msg->patch_array[i].image_roi.y_offset +
                          msg->patch_array[i].image_roi.height), 255, 20);
        }
        continue;
      }
    }
    pub_patch_array.publish(out_pa_msg);
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
    if (!listener_.canTransform("base_link",header.frame_id, camera_point_a.header.stamp)) {
      ROS_INFO("Couldn't transform base_link to %s\n",camera_point_a.header.frame_id.c_str());
      return false;
    }
    listener_.lookupTransform("base_link",header.frame_id,ros::Time(0),camera_transform);
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
      try
      {
          listener_.transformPoint("odom",ground_point,ground_point);
      }
      catch(const std::exception& e)
      {
          ROS_ERROR_STREAM("Unable to transform ground_point to \"odom\": " << e.what());
          return false;
      }
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

  // Take the plane model fit by the pcl_ros nodelets
  void planeModelCallback(const pcl_msgs::ModelCoefficientsPtr& msg)
  {
    visualization_msgs::Marker mark;
    if (msg->values.size() != 4) {
      ROS_DEBUG("Invalid Plane Fit");
      miss_count_ += 1;
      if (miss_count_ > 15) {
        ground_plane_ << 0.,0.,1.,0.;
        mark.color.r = 1.0;
        mark.color.g = 0.0;
        mark.color.b = 0.0;
      }
      else {
        mark.color.r = 0.0;
        mark.color.g = 1.0;
        mark.color.b = 1.0;
      }
    }
    else {
      miss_count_ = 0;
      // Keep the plane around for patch projection, publish a plane normal
      // marker for Rviz
      ROS_DEBUG("Model Coefficients: %f, %f, %f, %f",msg->values[0],
                                                    msg->values[1],
                                                    msg->values[2],
                                                    msg->values[3]);
      ground_plane_ << msg->values[0],
                    msg->values[1],
                    msg->values[2],
                    msg->values[3];
      mark.color.r = 0.0;
      mark.color.g = 0.5;
      mark.color.b = 1.0;
    }
    mark.header = msg->header;
    mark.header.frame_id = "/base_link";
    Eigen::Quaterniond orientation;
    Eigen::Vector3d xhat;
    xhat.setZero();
    xhat[0] = 1.0;
    orientation.setFromTwoVectors(xhat, ground_plane_.head(3));
    tf::quaternionEigenToMsg(orientation, mark.pose.orientation);
    mark.scale.x = 1.0;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    mark.color.a = 1.0;
    pub_marker.publish(mark);
  }

  void configCallback(saliency_detector::ground_projector_paramsConfig &config, uint32_t level)
  {
    min_major_axis_ = config.min_major_axis;
    max_major_axis_ = config.max_major_axis;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_projector");
  GroundProjectorNode gp;
  ros::spin();
}
