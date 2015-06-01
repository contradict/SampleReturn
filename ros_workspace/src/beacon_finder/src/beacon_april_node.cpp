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
  int min_tag_size_;
 protected:
  std::string famname_;
  apriltag_family_t *tag_fam_;
  apriltag_detector_t *tag_det_;
  image_geometry::PinholeCameraModel model_;
  std::vector<double> covariance_;
  double position_sigma_;
  double position_sigma_scale_;
  double rotation_sigma_;
  double rotation_sigma_3tag_;
  double rot_thresh_;
  double pos_thresh_;
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

  pnh.param("min_tag_size", min_tag_size_, 100);

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
  pnh.param("rotation_sigma_3tag", this->rotation_sigma_3tag_, 0.15);

  pnh.param("consensus_rot_threshold", this->rot_thresh_, 0.1);
  pnh.param("consensus_trans_threshold", this->pos_thresh_, 1.0);

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
  
  ros::Time start_time = ros::Time::now();
  
  image_u8_t *apr_image = image_u8_create(msg->width, msg->height);
  cv::Mat gray(msg->height, msg->width, CV_8UC1, apr_image->buf, apr_image->stride);
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

  zarray_t *detections;
  detections = apriltag_detector_detect(tag_det_, apr_image);
  
  //report number of tags if any found
  if (zarray_size(detections) > 1) {
    
    ROS_DEBUG_NAMED("tag_detection", "APRIL BEACON FINDER found %d tags.", zarray_size(detections));
    double detection_time = (ros::Time::now() - start_time).toSec();
    ROS_DEBUG("APRIL BEACON FINDER tag detection executed in: %lf", detection_time);
  
    //if less than 3, don't solve
    if (zarray_size(detections) >= 3) {
      
      model_.fromCameraInfo(cam_info);
      
      geometry_msgs::PoseArray tag_pose_array;
      tag_pose_array.header = cv_ptr->header;
      std::vector<cv::Vec3d>beacon_positions;
      std::vector<cv::Vec3d>beacon_rotations;
      std::vector<geometry_msgs::Pose>beacon_poses;
      std::vector<cv::Point2d> all_imgPts;
      std::vector<cv::Point3d> transformed_corners;
    
      for (int i = 0; i < zarray_size(detections); i++) {
          apriltag_detection_t *det;
          zarray_get(detections, i, &det);
          std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(det->id);
          if(description_itr == descriptions_.end()){
              ROS_WARN_THROTTLE(10.0, "APRIL BEACON FINDER Found tag: %d, but no description was found for it", det->id);
              continue;
          }
          AprilTagDescription description = description_itr->second;
          double tag_size = description.size();
          
        std::string frame_id = description.frame_name();
        std::string tf_err;
        if(!_tf.canTransform(frame_id, "beacon", ros::Time(0), &tf_err))
        {
            ROS_ERROR_STREAM("APRIL BEACON FINDER Unable to transform to frame " << frame_id << ": " << tf_err);
            continue;
        }
    
        tf::StampedTransform T_beacon_to_tag;
        _tf.lookupTransform(frame_id, "beacon", ros::Time(0), T_beacon_to_tag);
        
        double width, height;
        width = std::min(abs(det->p[1][0] - det->p[0][0]),
                         abs(det->p[2][0] - det->p[3][0]));
        height = std::min(abs(det->p[3][1] - det->p[0][1]),
                          abs(det->p[2][1] - det->p[1][1]));
    
        //stuff points from detections into an array for opencv, even if the tag is small,
        //these may be useful to the multi tag solver
        std::vector<cv::Point2d> imgPts;
        for(int i=0;i<4;i++)
        {
            cv::Point2d pt(det->p[i][0], det->p[i][1]);
            drawPoint(cv_ptr->image, pt);
            imgPts.push_back(pt);
            
            //also create full list, and transformed points for full check
            all_imgPts.push_back(pt);
            std_msgs::Header header;
            header.frame_id = frame_id;
            header.stamp = ros::Time(0);
            geometry_msgs::Point point;
            point.x = description.corners[i].x;
            point.y = description.corners[i].y;
            point.z = description.corners[i].z;
            geometry_msgs::PointStamped point_stamped;
            point_stamped.header = header;
            point_stamped.point = point;
            geometry_msgs::PointStamped transformed_point;
            _tf.transformPoint("beacon", point_stamped, transformed_point);
            cv::Point3d pt3d(transformed_point.point.x,
                             transformed_point.point.y,
                             transformed_point.point.z);
            transformed_corners.push_back(pt3d);
        }
      
      } //end detections iteration
        
      //try the multi_tag (3+) solution
      cv::Vec3d rvec, tvec;
      bool solved = false;
      solved = cv::solvePnP(transformed_corners, all_imgPts, model_.fullIntrinsicMatrix(), model_.distortionCoeffs(), rvec, tvec, false, CV_ITERATIVE);
      if ( !solved) {
          ROS_ERROR_NAMED("tag_detection", "APRIL BEACON FINDER Unable to solve for multiple tag pose.");
          ROS_ERROR_STREAM_NAMED("tag_detection", "APRIL BEACON FINDER corners:\n" << transformed_corners << std::endl << "imgPts:\n" << all_imgPts);
      } else if (tvec[2] < 0) {
          ROS_ERROR_NAMED("tag_detection", "APRIL BEACON FINDER solved for crazy pose! retrying");
          //sometimes we end up on the back side of the lens, so flip to the front and re-slolve
          tvec[0] *= -1;
          tvec[1] *= -1;
          tvec[2] *= -1;
          rvec[0] = 0;
          rvec[1] = 0;
          rvec[2] = 0;
          solved = cv::solvePnP(transformed_corners, all_imgPts, model_.fullIntrinsicMatrix(), model_.distortionCoeffs(), rvec, tvec, true, CV_ITERATIVE);
          if (tvec[2] < 0) {
              ROS_ERROR_NAMED("tag_detection", "APRIL BEACON FINDER solved for crazy pose a second time! giving up");
              solved = false;
          }
      }
    
      if ( !solved) {//still not solved
          ROS_ERROR_NAMED("tag_detection", "APRIL BEACON FINDER Unable to solve for multiple tag pose.");
          ROS_ERROR_STREAM_NAMED("tag_detection", "APRIL BEACON FINDER corners:\n" << transformed_corners << std::endl << "imgPts:\n" << all_imgPts);
      } else {
          ROS_DEBUG_STREAM_NAMED("tag_detection", "APRIL BEACON FINDER found solution for: "<< zarray_size(detections) << " tags.");        
        
          //this solution is for points xformed into beacon frame
          double th = cv::norm(rvec);
          cv::Vec3d axis;
          cv::normalize(rvec, axis);
          tf::Transform beacon_to_camera(tf::Quaternion(tf::Vector3(axis[0], axis[1], axis[2]), th),
                  tf::Vector3(tvec[0], tvec[1], tvec[2]));
    
          geometry_msgs::PoseStamped beacon_pose;
          tf::pointTFToMsg( beacon_to_camera.getOrigin(), beacon_pose.pose.position);
          tf::quaternionTFToMsg( beacon_to_camera.getRotation(), beacon_pose.pose.orientation);
          beacon_pose.header = cv_ptr->header;
          
          //put this pose in the array, and the debug msg    
          tag_pose_array.poses.push_back(beacon_pose.pose);
          beacon_debug_pose_pub_.publish(beacon_pose);           
    
            geometry_msgs::PoseWithCovarianceStamped beacon_pose_msg;
            beacon_pose_msg.header = cv_ptr->header;
      
            //calculate covariance based on range
            cv::Vec3d pos(beacon_pose.pose.position.x,
                          beacon_pose.pose.position.y,
                          beacon_pose.pose.position.z);
            double range = cv::norm(pos);
            double pos_sigma = position_sigma_scale_ * range + position_sigma_;
            double pos_covariance = pos_sigma * pos_sigma;
            double rot_covariance = rotation_sigma_3tag_ * rotation_sigma_3tag_;
            covariance_[0] = pos_covariance;
            covariance_[7] = pos_covariance;
            covariance_[14] = pos_covariance;
            covariance_[21] = rot_covariance;
            covariance_[28] = rot_covariance;
            covariance_[35] = rot_covariance;
            std::copy(covariance_.begin(), covariance_.end(), beacon_pose_msg.pose.covariance.begin());
            beacon_pose_msg.pose.pose = beacon_pose.pose;
      
            ROS_DEBUG_STREAM_NAMED("tag_detection", "APRIL BEACON FINDER publishing " << zarray_size(detections) << " tag solution.");        
            
            beacon_pose_pub_.publish(beacon_pose_msg);
      }
  
      //publish tag poses for debug purposes
      tag_pose_pub_.publish(tag_pose_array);
    
    } //end if >= 3
    
  } //end if > 1
  
  //free up detections memory
  apriltag_detections_destroy(detections);
  
  //publish debug image
  image_pub_.publish(cv_ptr->toImageMsg());

  double execution_time = (ros::Time::now() - start_time).toSec();
  
  ROS_DEBUG("APRIL BEACON FINDER full image callback executed in: %lf", execution_time);
  
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
                    ROS_ERROR_STREAM("APRIL BEACON FINDER Non-numeric type in corners at index " << i << ": " << v.getType());
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
    ROS_INFO_STREAM("APRIL BEACON FINDER Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
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
