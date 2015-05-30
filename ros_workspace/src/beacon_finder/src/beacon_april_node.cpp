#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <XmlRpcException.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"

#include <beacon_finder/AprilTagDetection.h>
#include <beacon_finder/AprilTagDetectionArray.h>

namespace beacon_april_node{
 
class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name, std::vector<cv::Point3d> &corner_pos):id_(id), size_(size), frame_name_(frame_name), corners(corner_pos) {}
  double size(){return size_;}
  int id(){return id_;} 
  std::string& frame_name(){return frame_name_;}
  std::vector<cv::Point3d> corners;
 private:
  int id_;
  double size_;
  std::string frame_name_;
}; 
  
class BeaconAprilDetector{
 public:
  BeaconAprilDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~BeaconAprilDetector();
 
 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);
  void drawPoint(cv::Mat &img, const cv::Point2d pt);

 private:
  std::map<int, AprilTagDescription> descriptions_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher tag_pose_pub_;
  ros::Publisher beacon_pose_pub_;
  ros::Publisher beacon_debug_pose_pub_;
  tf::TransformListener    _tf;
  cv::RNG rng_;
  int solve_tries_;
  double solve_noise_;
  double rvec_tolerance_;
  int point_size_;
 protected:
  std::string famname_;
  apriltag_family_t *tag_fam_;
  apriltag_detector_t *tag_det_;
  image_geometry::PinholeCameraModel model_;
  std::vector<double> covariance_;
  double position_sigma_;
  double position_sigma_scale_;
  double rotation_sigma_;
};

BeaconAprilDetector::BeaconAprilDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    it_(nh),
    _tf(ros::Duration(10.0)),
    tag_det_(NULL),
    covariance_(36,0.0),
    rng_(0),
    point_size_(10)
{
  //get april tag descriptors from launch file
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  pnh.param("solve_tries", solve_tries_, 3);
  pnh.param("solve_noise", solve_noise_, 1e-3);
  pnh.param("rvec_tolerance", rvec_tolerance_, 1e-2);

  //get tag family parametre
  pnh.param("tag_family", famname_, std::string("tag36h11"));
  if (!famname_.compare("tag36h11"))
    this->tag_fam_ = tag36h11_create();
  else if (!famname_.compare("tag36h10"))
    this->tag_fam_ = tag36h10_create();
  else if (!famname_.compare("tag36artoolkit"))
    this->tag_fam_ = tag36artoolkit_create();
  else if (!famname_.compare("tag25h9"))
    this->tag_fam_ = tag25h9_create();
  else if (!famname_.compare("tag25h7"))
    this->tag_fam_ = tag25h7_create();
  else {
    ROS_ERROR("Unrecognized tag family name %s. Use e.g. \"tag36h11\".", famname_.c_str());
    return;
  }

  int border;
  pnh.param("border", border, 1);
  this->tag_fam_->black_border = border;

  //setup tag detector
  this->tag_det_ = apriltag_detector_create();
  apriltag_detector_add_family(this->tag_det_, this->tag_fam_);
  //these defaults taken from apriltag_demo.c
  double decim, blur;
  pnh.param("decimate", decim, 1.0);
  this->tag_det_->quad_decimate = decim;
  pnh.param("blur", blur, 0.0);
  this->tag_det_->quad_sigma = blur;
  pnh.param("threads", this->tag_det_->nthreads, 4);
  pnh.param("debug", this->tag_det_->debug, 0);
  pnh.param("refine_edges", this->tag_det_->refine_edges, 1);
  pnh.param("refine_decode", this->tag_det_->refine_decode, 0);
  pnh.param("refine_pose", this->tag_det_->refine_pose, 0);
  pnh.param("position_sigma", this->position_sigma_, 0.01);
  pnh.param("position_sigma_scale", this->position_sigma_scale_, 0.05);
  pnh.param("rotation_sigma", this->rotation_sigma_, 0.15);

  image_sub_ = it_.subscribeCamera("image", 1, &BeaconAprilDetector::imageCb, this);
  image_pub_ = it_.advertise("tag_detections_image", 1);
  detections_pub_ = pnh.advertise<beacon_finder::AprilTagDetectionArray>("tag_detections", 1);
  tag_pose_pub_ = pnh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
  beacon_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("beacon_pose", 10);
  beacon_debug_pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("beacon_pose_debug", 10);

}

BeaconAprilDetector::~BeaconAprilDetector(){
  image_sub_.shutdown();
  if(tag_det_ != NULL)
  {
      apriltag_detector_destroy(tag_det_);
      if (!famname_.compare("tag36h11"))
          tag36h11_destroy(tag_fam_);
      else if (!famname_.compare("tag36h10"))
          tag36h10_destroy(tag_fam_);
      else if (!famname_.compare("tag36artoolkit"))
          tag36artoolkit_destroy(tag_fam_);
      else if (!famname_.compare("tag25h9"))
          tag25h9_destroy(tag_fam_);
      else if (!famname_.compare("tag25h7"))
          tag25h7_destroy(tag_fam_);
  }
}

void BeaconAprilDetector::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info){
  if(tag_det_ == NULL)
    return;
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  image_u8_t *apr_image = image_u8_create(msg->width, msg->height);
  cv::Mat gray(msg->height, msg->width, CV_8UC1, apr_image->buf, apr_image->stride);
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

  zarray_t *detections;
  detections = apriltag_detector_detect(tag_det_, apr_image);
  /*
  double fx = cam_info->K[0];
  double fy = cam_info->K[4];
  double px = cam_info->K[2];
  double py = cam_info->K[5];
  */
  model_.fromCameraInfo(cam_info);

  beacon_finder::AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header = cv_ptr->header;

  ROS_DEBUG_NAMED("apriltags", "BEACON FINDER found %d tags.", zarray_size(detections));

  for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(det->id);
      if(description_itr == descriptions_.end()){
          ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", det->id);
          continue;
      }
      AprilTagDescription description = description_itr->second;
      double tag_size = description.size();
      
    std::string frame_id = description.frame_name();
    std::string tf_err;
    if(!_tf.canTransform(frame_id, "beacon", ros::Time(0), &tf_err))
    {
        ROS_ERROR_STREAM("Unable to transform to frame " << frame_id << ": " << tf_err);
        continue;
    }

    tf::StampedTransform T_beacon_to_tag;
    _tf.lookupTransform(frame_id, "beacon", ros::Time(0), T_beacon_to_tag);
        
    std::vector<cv::Point2d> imgPts;
    for(int i=0;i<4;i++)
    {
        cv::Point2d pt(det->p[i][0], det->p[i][1]);
        drawPoint(cv_ptr->image, pt);
        imgPts.push_back(pt);
    }
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    for(int i=0;i<solve_tries_;i++)
    {
        cv::Vec3d rvec, tvec;
        std::vector<cv::Point2d> disturbed_imgPts;
        for(const auto &pt : imgPts)
        {
            disturbed_imgPts.push_back(pt + cv::Point2d(solve_noise_*(double)rng_, solve_noise_*(double)rng_));
        }
        if (cv::solvePnP(description.corners, imgPts, model_.fullIntrinsicMatrix(), model_.distortionCoeffs(), rvec, tvec, false, CV_ITERATIVE) == false)
        {
            ROS_ERROR_NAMED("solver", "Unable to solve for tag pose.");
            ROS_ERROR_STREAM_NAMED("solver", "corners:\n" << description.corners << std::endl << "imagPts:\n" << imgPts);
            continue;
        }
        ROS_DEBUG_STREAM_NAMED("solver", "noisy rvec(" << i << "): " << rvec);
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }
    double distance=0;
    cv::Vec3d rvec=rvecs[0], tvec=tvecs[0];
    for(int i=1; i<solve_tries_; i++)
    {
        distance=std::max(distance, cv::norm(rvecs[0] - rvecs[i]));
        rvec += rvecs[i];
        tvec += tvecs[i];
    }
    tvec /= solve_tries_;
    rvec /= solve_tries_;
    
    ROS_DEBUG_STREAM_NAMED("solver", "Tag: " << frame_id << " rvec:(" << rvec[0] << ", " << rvec[1] << ", " << rvec[2] << ")");
    ROS_DEBUG_STREAM_NAMED("solver", "    tvec:(" << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << ")");
    ROS_DEBUG_STREAM_NAMED("solver", "imgPts:\n" <<
        "[" << imgPts[0] << ",\n" <<
        " " << imgPts[1] << ",\n" <<
        " " << imgPts[2] << ",\n" <<
        " " << imgPts[3] << "]");
    if(distance>rvec_tolerance_)
    {
        ROS_ERROR_STREAM_NAMED("solver", "large rvec distance, skipping: " << distance);
        continue;
    }
    double th = cv::norm(rvec);
    cv::Vec3d axis;
    cv::normalize(rvec, axis);
    tf::Transform tag_to_camera(tf::Quaternion(tf::Vector3(axis[0], axis[1], axis[2]), th),
            tf::Vector3(tvec[0], tvec[1], tvec[2]));

    tf::StampedTransform tag_to_camera_s(tag_to_camera,
            msg->header.stamp, frame_id, msg->header.frame_id);
    geometry_msgs::PoseStamped tag_pose;
    tf::pointTFToMsg( tag_to_camera.getOrigin(), tag_pose.pose.position);
    tf::quaternionTFToMsg( tag_to_camera.getRotation(), tag_pose.pose.orientation);
    tag_pose.header = cv_ptr->header;

    tf::StampedTransform beacon_to_camera(  tag_to_camera_s* T_beacon_to_tag,
            msg->header.stamp, "beacon", msg->header.frame_id);
    geometry_msgs::PoseStamped beacon_pose;
    tf::pointTFToMsg( beacon_to_camera.getOrigin(), beacon_pose.pose.position);
    tf::quaternionTFToMsg( beacon_to_camera.getRotation(), beacon_pose.pose.orientation);
    beacon_pose.header = cv_ptr->header;

    //calculate covariance based on range
    double range = cv::norm(tvec);
    double pos_sigma = position_sigma_scale_ * range + position_sigma_;
    double pos_covariance = pos_sigma * pos_sigma;
    double rot_covariance = rotation_sigma_ * rotation_sigma_;
    covariance_[0] = pos_covariance;
    covariance_[7] = pos_covariance;
    covariance_[14] = pos_covariance;
    covariance_[21] = rot_covariance;
    covariance_[28] = rot_covariance;
    covariance_[35] = rot_covariance;

    geometry_msgs::PoseWithCovarianceStamped beacon_pose_msg;
    beacon_pose_msg.header = cv_ptr->header;
    std::copy(covariance_.begin(), covariance_.end(), beacon_pose_msg.pose.covariance.begin());
    beacon_pose_msg.pose.pose = beacon_pose.pose;

    beacon_finder::AprilTagDetection tag_detection;
    tag_detection.header = cv_ptr->header;
    tag_detection.pose = tag_pose.pose;
    tag_detection.id = det->id;
    tag_detection.size = tag_size;
    tag_detection_array.detections.push_back(tag_detection);
    tag_pose_array.poses.push_back(tag_pose.pose);

    beacon_pose_pub_.publish(beacon_pose_msg);
    beacon_debug_pose_pub_.publish(beacon_pose);
  }
  //free up detections memory
  apriltag_detections_destroy(detections);

  detections_pub_.publish(tag_detection_array);
  tag_pose_pub_.publish(tag_pose_array);
  image_pub_.publish(cv_ptr->toImageMsg());
}

void BeaconAprilDetector::drawPoint(cv::Mat &img, const cv::Point2d pt)
{
    cv::Point2d xhat(point_size_, 0);
    cv::Point2d yhat(0, point_size_);

    cv::line(img, pt, pt+xhat, cv::Scalar(255, 0, 0));
    cv::line(img, pt, pt-xhat, cv::Scalar(255, 0, 0));
    cv::line(img, pt, pt+yhat, cv::Scalar(255, 0, 0));
    cv::line(img, pt, pt-yhat, cv::Scalar(255, 0, 0));
}


std::map<int, AprilTagDescription> BeaconAprilDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    std::vector<cv::Point3d> corners;
    if(tag_description.hasMember("corners")) {
        ROS_ASSERT(tag_description["corners"].getType() == XmlRpc::XmlRpcValue::TypeArray);
        XmlRpc::XmlRpcValue v = tag_description["corners"];
        for(int i =0; i < v.size(); i+=3)
        {
            auto parsenum = [&i](XmlRpc::XmlRpcValue v) -> double {
                if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    return (double)v;
                }
                else if(v.getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    return (int)v;
                }
                else
                {
                    ROS_ERROR_STREAM("Non-numeric type in corners at index " << i << ": " << v.getType());
                }
                return 0;
            };
            double x=0,y=0,z=0;
            x = parsenum(v[i]);
            y = parsenum(v[i+1]);
            z = parsenum(v[i+2]);
            corners.push_back(cv::Point3d(x, y, z));
         }
    }
    else {
        corners = {cv::Point3d(-size/2, -size/2, 0),
                          cv::Point3d(size/2, -size/2, 0),
                          cv::Point3d(size/2, size/2, 0),
                          cv::Point3d(-size/2, size/2, 0)};
    }
    AprilTagDescription description(id, size, frame_name, corners);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}


int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  beacon_april_node::BeaconAprilDetector detector(nh, pnh);
  ros::spin();
}
