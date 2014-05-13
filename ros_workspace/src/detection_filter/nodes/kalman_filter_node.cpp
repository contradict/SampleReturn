#include <stdio.h>
#include <memory>
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

  std::vector<std::shared_ptr<cv::KalmanFilter> > filter_list_;
  ros::Time last_time_;
  float max_dist_;

  image_geometry::PinholeCameraModel cam_model_;

  public:
  KalmanDetectionFilter()
  {
    cam_info_topic = "/cameras/manipulator/left/camera_info";
    detection_topic = "/img_point";

    ros::NodeHandle private_node_handle_("~");

    sub_cam_info =
      nh.subscribe(cam_info_topic.c_str(), 3, &KalmanDetectionFilter::cameraInfoCallback, this);

    sub_detection =
      nh.subscribe(detection_topic.c_str(), 3, &KalmanDetectionFilter::detectionCallback, this);

    max_dist_ = 100.0;
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
    cv::setIdentity(KF->processNoiseCov, cv::Scalar(5e0));
    cv::setIdentity(KF->measurementNoiseCov, cv::Scalar(2e1));
    cv::setIdentity(KF->errorCovPost, cv::Scalar(1e1));

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
    drawFilterStates();
  }

  void drawFilterStates() {
    std::cout << "Number of Filters: " << filter_list_.size() << std::endl;
    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    for (auto filter_ptr : filter_list_){
      cv::Point mean(filter_ptr->statePost.at<float>(0),
          filter_ptr->statePost.at<float>(1));
      float rad_x = filter_ptr->errorCovPost.at<float>(0,0);
      float rad_y = filter_ptr->errorCovPost.at<float>(1,1);
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
      std::cout << "Cov Post: " << filter_ptr->errorCovPost << std::endl;
      std::cout << "Cov Pre: " << filter_ptr->errorCovPre << std::endl;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_detection_filter");
  KalmanDetectionFilter kdf;
  ros::spin();
}
