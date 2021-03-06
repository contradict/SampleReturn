#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <samplereturn_msgs/PatchArray.h>
#include <samplereturn_msgs/Patch.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <samplereturn/mask_utils.h>

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
  ros::Publisher pub_frustum;
  std::string sub_patch_array_topic;
  std::string pub_patch_array_topic;
  std::string sub_plane_model_topic;
  std::string pub_debug_image_topic;
  std::string marker_topic;
  std::string point_topic;

  tf::TransformListener listener_;

  dynamic_reconfigure::Server<saliency_detector::ground_projector_paramsConfig> dr_srv;

  double min_major_axis_, max_major_axis_;
  int miss_count_;
  int frustum_buffer_;

  Eigen::Vector4d ground_plane_;
  image_geometry::PinholeCameraModel cam_model_;

  public:
  GroundProjectorNode() {
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<saliency_detector::ground_projector_paramsConfig>::CallbackType cb;
    cb = boost::bind(&GroundProjectorNode::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("frustum_buffer_", frustum_buffer_, 200);

    sub_patch_array =
      nh.subscribe("patch_array", 1, &GroundProjectorNode::patchArrayCallback, this);

    sub_plane_model =
      nh.subscribe("model", 1, &GroundProjectorNode::planeModelCallback, this);

    pub_patch_array =
      nh.advertise<samplereturn_msgs::PatchArray>("projected_patch_array", 1);

    pub_marker =
      nh.advertise<visualization_msgs::Marker>("ground_marker", 1);

    pub_point =
      nh.advertise<geometry_msgs::PointStamped>("ground_point", 1);

    pub_debug_image =
      nh.advertise<sensor_msgs::Image>("projector_debug_image", 1);

    pub_frustum =
      nh.advertise<geometry_msgs::PolygonStamped>("view_frustum", 3);

    ground_plane_ << 0.,0.,1.,0.;
    miss_count_ = 0;
  }

  // Use ground plane and camera info to publish a view frustum on the ground
  // plane. This should be called by the plane update, and is for visualization.
  void publishFrustum(pcl_msgs::ModelCoefficients plane)
  {
    if (!cam_model_.initialized()) {
      return;
    }
    sensor_msgs::CameraInfo cam_info = cam_model_.cameraInfo();
    if(!listener_.canTransform("base_link", cam_info.header.frame_id, cam_info.header.stamp))
    {
        return;
    }
    // Take corners of image, buffer inward by param
    std::vector<cv::Point2d> corners, rect_corners;
    corners.push_back(cv::Point2d(0 + frustum_buffer_,
          0 + frustum_buffer_));
    corners.push_back(cv::Point2d(0 + frustum_buffer_,
          cam_info.height - frustum_buffer_));
    corners.push_back(cv::Point2d(cam_info.width - frustum_buffer_,
          cam_info.height - frustum_buffer_));
    corners.push_back(cv::Point2d(cam_info.width - frustum_buffer_,
          0 + frustum_buffer_));
    // Rectify points
    rect_corners.resize(corners.size());
    std::transform(corners.begin(), corners.end(), rect_corners.begin(),
        [this](cv::Point2d uv) -> cv::Point2d
        {
          cv::Point2d rect_uv = cam_model_.rectifyPoint(uv);
          return rect_uv;
        });
    // Project to ray, intersect with ground plane
    std::vector<Eigen::Vector3d> base_link_rays;
    base_link_rays.resize(corners.size());
    std::transform(corners.begin(), corners.end(), base_link_rays.begin(),
        [cam_info, plane, this](cv::Point2d uv) -> Eigen::Vector3d
        {
          cv::Point3d xyz = cam_model_.projectPixelTo3dRay(uv);
          tf::Stamped<tf::Vector3> vect(tf::Vector3(xyz.x, xyz.y, xyz.z),
              plane.header.stamp,
              cam_info.header.frame_id);
          tf::Stamped<tf::Vector3> vect_t;
          listener_.transformVector("base_link", vect, vect_t);
          Eigen::Vector3d ray;
          tf::vectorTFToEigen(vect_t, ray);
          return ray;
        });
    // Get ray origin, shared for all rays
    Eigen::Vector3d ray_origin;
    tf::Vector3 pos;
    tf::StampedTransform camera_transform;
    listener_.lookupTransform("base_link", cam_info.header.frame_id,
        ros::Time(0), camera_transform);
    pos = camera_transform.getOrigin();
    tf::vectorTFToEigen(pos, ray_origin);
    // These are the ground points in the base_link frame
    std::vector<Eigen::Vector3d> ground_points;
    ground_points.resize(base_link_rays.size());
    std::transform(base_link_rays.begin(), base_link_rays.end(), ground_points.begin(),
        [ray_origin, this](Eigen::Vector3d ray) -> Eigen::Vector3d
        {
          Eigen::Vector3d ground_point = samplereturn::intersectRayPlane(ray, ray_origin, ground_plane_);
          return ground_point;
        });
    // Publish polygon of points
    geometry_msgs::PolygonStamped ground_polygon;
    ground_polygon.header.stamp = plane.header.stamp;
    ground_polygon.header.frame_id = plane.header.frame_id;
    for (auto pt : ground_points)
    {
      geometry_msgs::Point32 p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];
      ground_polygon.polygon.points.push_back(p);
    }

    pub_frustum.publish(ground_polygon);
  }

  // For a set of candidate sample patches, project them onto the ground to get
  // 3D location and size estimate. If stereo data is available, usse that estimated
  // ground plane, otherwise assume the plane under the wheels.
  void patchArrayCallback(const samplereturn_msgs::PatchArrayConstPtr& msg)
  {
    bool enable_debug = (pub_debug_image.getNumSubscribers() != 0);
    samplereturn_msgs::PatchArray out_pa_msg;
    out_pa_msg.header = msg->header;
    out_pa_msg.cam_info = msg->cam_info;
    cam_model_.fromCameraInfo(msg->cam_info);

    cv::Mat debug_image;

    if (enable_debug) {
      debug_image = cv::Mat::ones(msg->cam_info.height,
          msg->cam_info.width, CV_8U)*255;
    }

    // For each patch, project onto the available ground plane and make
    // accurate 3D position and size estimates. Filter out candidate patches
    // that are too large or small.
    // Take the saliency contour, find the major axis, cast rays to
    // the ground plane
    cv_bridge::CvImagePtr cv_ptr;
    for (size_t i = 0; i < msg->patch_array.size(); i++) {
      try {
        cv_ptr = cv_bridge::toCvCopy(msg->patch_array[i].mask, "");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

      if (enable_debug) {
        cv_ptr->image.copyTo(debug_image(cv::Rect(msg->patch_array[i].image_roi.x_offset,
                msg->patch_array[i].image_roi.y_offset,
                msg->patch_array[i].image_roi.width,
                msg->patch_array[i].image_roi.height)));
      }

      cv::Point2f roi_offset(msg->patch_array[i].image_roi.x_offset,
              msg->patch_array[i].image_roi.y_offset);

      float dimension, angle;
      tf::Stamped<tf::Point> world_point;
      if(!samplereturn::computeMaskPositionAndSize(listener_,
              cv_ptr->image, roi_offset,
              cam_model_, msg->header.stamp, msg->header.frame_id,
              ground_plane_, "base_link",
              &dimension, &angle, &world_point,
              enable_debug?&debug_image:NULL))
      {
          continue;
      }

      // if sample size is within bounds, add this point to the output set
      if ((dimension < max_major_axis_) &&
          (dimension > min_major_axis_)) {
        try
        {
            listener_.transformPoint("odom",world_point,world_point);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("Unable to transform ground_point to \"odom\": " << e.what());
            continue;
        }
        samplereturn_msgs::Patch pa_msg;
        pa_msg.image = msg->patch_array[i].image;
        pa_msg.mask = msg->patch_array[i].mask;
        pa_msg.image_roi = msg->patch_array[i].image_roi;
        tf::pointTFToMsg(world_point, pa_msg.world_point.point);
        pa_msg.world_point.header.frame_id = "odom";
        pa_msg.world_point.header.stamp = msg->header.stamp;
        pub_point.publish(pa_msg.world_point);
        out_pa_msg.patch_array.push_back(pa_msg);
      }
      else {
        if (enable_debug) {
          cv::line(debug_image,
              cv::Point2f(msg->patch_array[i].image_roi.x_offset,
                          msg->patch_array[i].image_roi.y_offset),
              cv::Point2f(msg->patch_array[i].image_roi.x_offset +
                          msg->patch_array[i].image_roi.width,
                          msg->patch_array[i].image_roi.y_offset +
                          msg->patch_array[i].image_roi.height), 255, 20);
          cv::line(debug_image,
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
    if (enable_debug) {
      sensor_msgs::ImagePtr debug_image_msg =
        cv_bridge::CvImage(msg->header,"mono8",debug_image).toImageMsg();
      pub_debug_image.publish(debug_image_msg);
    }
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

    // Compute and publish the view frustum, given the ground plane
    publishFrustum(*msg);
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
