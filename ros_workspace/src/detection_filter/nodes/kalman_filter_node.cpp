#include <stdio.h>
#include <memory>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <samplereturn_msgs/NamedPoint.h>

/* This is going to subscribe to a detection channel, maintain some number
 * of Kalman filters for hypothesis, and publish confirmed detections
 * when the covariance and velocity of a filter fall below a threshold.
 * It will also age out filters that have too large a convariance.
 */

class KalmanDetectionFilter
{
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_cam_info;
  ros::Publisher pub_detection;

  std::string cam_info_topic;
  std::string detection_topic;
  std::string filtered_detection_topic;

  std::vector<std::shared_ptr<cv::KalmanFilter> > filter_list_;
  ros::Time last_time_;
  double max_dist_;
  double max_cov_;
  double max_pub_cov_;
  double max_pub_vel_;

  image_geometry::PinholeCameraModel cam_model_;

  public:
  KalmanDetectionFilter()
  {
    cam_info_topic = "/cameras/manipulator/left/camera_info";
    detection_topic = "point";
    filtered_detection_topic = "filtered_point";

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("max_dist", max_dist_, double(100.0));
    private_node_handle_.param("max_cov", max_cov_, double(10));
    private_node_handle_.param("max_pub_cov_", max_pub_cov_, double(5.0));
    private_node_handle_.param("max_pub_vel_", max_pub_vel_, double(0.02));

    sub_cam_info =
      nh.subscribe(cam_info_topic.c_str(), 3, &KalmanDetectionFilter::cameraInfoCallback, this);

    sub_detection =
      nh.subscribe(detection_topic.c_str(), 3, &KalmanDetectionFilter::detectionCallback, this);

    pub_detection =
      nh.advertise<samplereturn_msgs::NamedPoint>(filtered_detection_topic.c_str(), 3);

    last_time_.sec = 0.0;
    last_time_.nsec = 0.0;
  }

  void detectionCallback(const samplereturn_msgs::NamedPoint& msg)
  {
    //ROS_INFO("Detection Callback");
    if (filter_list_.size() == 0) {
      addFilter(msg);
      return;
    }
    if (last_time_ < msg.header.stamp) {
      last_time_ = msg.header.stamp;
      for (int i=0; i<filter_list_.size(); i++) {
        filter_list_[i]->predict();
      }
    }
    checkObservation(msg);
    publishFilteredDetections(msg);
  }

  void publishFilteredDetections(const samplereturn_msgs::NamedPoint& msg) {
    for (auto filter_ptr : filter_list_) {
      cv::Mat eigenvalues;
      cv::eigen(filter_ptr->errorCovPost, eigenvalues);
      for (int i=0; i<eigenvalues.rows; i++) {
        if (eigenvalues.at<float>(i) > max_pub_cov_) {
          break;
        }
      }
      if (filter_ptr->statePost.at<float>(2) < max_pub_vel_ ||
          filter_ptr->statePost.at<float>(3) < max_pub_vel_) {
        samplereturn_msgs::NamedPoint point_msg;
        point_msg.header.frame_id = "/odom";
        point_msg.header.stamp = ros::Time::now();
        point_msg.grip_angle = msg.grip_angle;
        point_msg.sample_id = msg.sample_id;
        point_msg.point.x = filter_ptr->statePost.at<float>(0);
        point_msg.point.y = filter_ptr->statePost.at<float>(1);
        point_msg.point.z = 0;
        pub_detection.publish(point_msg);
      }
    }
  }

  void addFilter(const samplereturn_msgs::NamedPoint& msg)
  {
    std::shared_ptr<cv::KalmanFilter> KF (new cv::KalmanFilter(4,2));
    cv::Mat state(4, 1, CV_32F); /* x, y, vx, vy */
    cv::Mat processNoise(4, 1, CV_32F);

    KF->transitionMatrix = (cv::Mat_<float>(4,4) << 1, 0, 0.03, 0,
                                                    0, 1, 0, 0.03,
                                                    0, 0, 1, 0,
                                                    0, 0, 0, 1);
    cv::setIdentity(KF->measurementMatrix);
    cv::setIdentity(KF->processNoiseCov, cv::Scalar(1e-3));
    cv::setIdentity(KF->measurementNoiseCov, cv::Scalar(2e-2));
    cv::setIdentity(KF->errorCovPost, cv::Scalar(1e-1));

    KF->statePost.at<float>(0) = msg.point.x;
    KF->statePost.at<float>(1) = msg.point.y;
    KF->statePost.at<float>(2) = 0;
    KF->statePost.at<float>(3) = 0;

    KF->predict();
    filter_list_.push_back(KF);
    checkObservation(msg);
  }

  void addMeasurement(const cv::Mat meas_state, int filter_index)
  {
    ROS_INFO("Adding measurement to filter: %i", filter_index);
    filter_list_[filter_index]->correct(meas_state);
  }

  void checkObservation(const samplereturn_msgs::NamedPoint& msg)
  {
    cv::Mat meas_state(2, 1, CV_32F);
    meas_state.at<float>(0) = msg.point.x;
    meas_state.at<float>(1) = msg.point.y;

    for (int i=0; i<filter_list_.size(); i++) {
      cv::Mat dist = (filter_list_[i]->measurementMatrix)*(filter_list_[i]->statePost)
        - meas_state;
      std::cout << "Distance: " << cv::sum(dist)[0] << std::endl;
      if (cv::sum(dist)[0] < max_dist_) {
        addMeasurement(meas_state, i);
        return;
      }
    }
    addFilter(msg);
  }

  /* The process tick for all filters */
  void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    if (filter_list_.size() == 0) {
      return;
    }
    if (last_time_ < msg.header.stamp) {
      last_time_ = msg.header.stamp;
      for (int i=0; i<filter_list_.size(); i++) {
        filter_list_[i]->predict();
        filter_list_[i]->errorCovPre.copyTo(filter_list_[i]->errorCovPost);;
      }
    }
    checkFilterAges();
    drawFilterStates();
  }

  bool isOld (std::shared_ptr<cv::KalmanFilter> kf) {
    cv::Mat eigenvalues;
    cv::eigen(kf->errorCovPost, eigenvalues);
    for (int i=0; i<eigenvalues.rows; i++) {
      if (eigenvalues.at<float>(i) > max_cov_) {
        return true;
      }
    }
    return false;
  }

  void checkFilterAges() {
    filter_list_.erase(std::remove_if(filter_list_.begin(), filter_list_.end(),
        std::bind1st(std::mem_fun(&KalmanDetectionFilter::isOld),this)),
        filter_list_.end());
  }

  void drawFilterStates() {
    std::cout << "Number of Filters: " << filter_list_.size() << std::endl;
    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    float px_per_meter = 50.0;
    for (auto filter_ptr : filter_list_){
      cv::Point mean(filter_ptr->statePost.at<float>(0) * px_per_meter,
          filter_ptr->statePost.at<float>(1) * px_per_meter);
      float rad_x = filter_ptr->errorCovPost.at<float>(0,0) * px_per_meter;
      float rad_y = filter_ptr->errorCovPost.at<float>(1,1) * px_per_meter;
      cv::circle(img, mean, 5, cv::Scalar(255,0,0));
      cv::ellipse(img, mean, cv::Size(rad_x, rad_y), 0, 0, 360, cv::Scalar(0,255,0));
    }

    printFilterState();

    cv::imshow("Filter States", img);
    cv::waitKey(10);
  }

  void printFilterState() {
    for (auto filter_ptr : filter_list_) {
      std::cout << "State: " << filter_ptr->statePost << std::endl;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_detection_filter");
  KalmanDetectionFilter kdf;
  ros::spin();
}
