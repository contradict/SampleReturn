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
#include <samplereturn_msgs/PursuitResult.h>

/* This is going to subscribe to a detection channel, maintain some number
 * of Kalman filters for hypothesis, and publish confirmed detections
 * when the covariance and velocity of a filter fall below a threshold.
 * It will also age out filters that have too large a convariance.
 */

class KalmanDetectionFilter
{
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_img_detection;
  ros::Subscriber sub_cam_info;
  ros::Publisher pub_detection;
  ros::Publisher pub_img_detection;

  ros::Subscriber sub_ack;

  std::string cam_info_topic;
  std::string detection_topic;
  std::string img_detection_topic;
  std::string filtered_detection_topic;
  std::string filtered_img_detection_topic;
  std::string ack_topic;

  std::vector<std::shared_ptr<cv::KalmanFilter> > filter_list_;
  std::vector<std::shared_ptr<cv::KalmanFilter> > latched_filter_list_;
  // Exclusion sites are centers and radii (x,y,r)
  std::vector<std::tuple<float,float,float> > exclusion_list_;

  ros::Time last_time_;
  double max_dist_;
  double max_cov_;
  double max_pub_cov_;
  double max_pub_vel_;
  samplereturn_msgs::NamedPoint last_img_point_msg_;
  bool got_img_point_;

  bool accumulate_;
  double pos_exclusion_radius_;
  double neg_exclusion_radius_;

  image_geometry::PinholeCameraModel cam_model_;

  public:
  KalmanDetectionFilter()
  {
    //cam_info_topic = "camera_info";
    cam_info_topic = "/cameras/manipulator/left/camera_info";
    detection_topic = "point";
    img_detection_topic = "img_point";
    filtered_detection_topic = "filtered_point";
    filtered_img_detection_topic = "filtered_img_point";

    ack_topic = "ack";

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("max_dist", max_dist_, double(0.1));
    private_node_handle_.param("max_cov", max_cov_, double(1.0));
    private_node_handle_.param("max_pub_cov", max_pub_cov_, double(0.1));
    private_node_handle_.param("max_pub_vel", max_pub_vel_, double(0.02));

    private_node_handle_.param("accumulate", accumulate_, false);
    private_node_handle_.param("positive_exclusion_radius", pos_exclusion_radius_, double(10.0));
    private_node_handle_.param("negative_exclusion_radius", neg_exclusion_radius_, double(1.5));

    sub_cam_info =
      nh.subscribe(cam_info_topic.c_str(), 3, &KalmanDetectionFilter::cameraInfoCallback, this);

    sub_detection =
      nh.subscribe(detection_topic.c_str(), 3, &KalmanDetectionFilter::detectionCallback, this);

    sub_img_detection =
      nh.subscribe(img_detection_topic.c_str(), 3, &KalmanDetectionFilter::imgDetectionCallback, this);

    pub_detection =
      nh.advertise<samplereturn_msgs::NamedPoint>(filtered_detection_topic.c_str(), 3);

    pub_img_detection =
      nh.advertise<samplereturn_msgs::NamedPoint>(filtered_img_detection_topic.c_str(), 3);

    if(accumulate_) {
      sub_ack = nh.subscribe(ack_topic.c_str(), 3, &KalmanDetectionFilter::ackCallback, this);
    }

    last_time_.sec = 0.0;
    last_time_.nsec = 0.0;
  }

  /* For incoming detections: assign to filter or create new filter
   * For each filter, predict, (update), check for deletion
   * If accumulate_, move to list of points to inspect
   * Publish closest point to inspect
   * When done, delete and add exclusion zone, deleting other filters and inspection points
   */
  void ackCallback(const samplereturn_msgs::PursuitResult& msg)
  {
    float x,y,r;
    x = latched_filter_list_[0]->statePost.at<float>(0);
    y = latched_filter_list_[0]->statePost.at<float>(1);
    if (msg.success) {
      r = pos_exclusion_radius_;
    }
    else {
      r = neg_exclusion_radius_;
    }
    exclusion_list_.push_back(std::make_tuple(x,y,r));

    latched_filter_list_.erase(latched_filter_list_.begin());
  }

  void imgDetectionCallback(const samplereturn_msgs::NamedPoint& msg)
  {
    last_img_point_msg_ = msg;
    got_img_point_ = true;
  }

  void detectionCallback(const samplereturn_msgs::NamedPoint& msg)
  {
    if (filter_list_.size() == 0 && latched_filter_list_.size()==0) {
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

    if (!accumulate_) {
      publishFilteredDetections(msg);
    }
    else {
      processFilters();
    }
  }

  void publishTop() {
    if (latched_filter_list_.size() > 0) {
      samplereturn_msgs::NamedPoint point_msg;
      point_msg.header.frame_id = "/odom";
      point_msg.header.stamp = ros::Time::now();
      point_msg.point.x = latched_filter_list_[0]->statePost.at<float>(0);
      point_msg.point.y = latched_filter_list_[0]->statePost.at<float>(1);
      point_msg.point.z = 0;
      pub_detection.publish(point_msg);
    }
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

  void addMeasurement(const cv::Mat meas_state, int filter_index, bool latched)
  {
    ROS_DEBUG("Adding measurement to filter: %i", filter_index);
    if (latched) {
      latched_filter_list_[filter_index]->correct(meas_state);
    }
    else {
      filter_list_[filter_index]->correct(meas_state);
    }
  }

  void checkObservation(const samplereturn_msgs::NamedPoint& msg)
  {
    cv::Mat meas_state(2, 1, CV_32F);
    meas_state.at<float>(0) = msg.point.x;
    meas_state.at<float>(1) = msg.point.y;

    for (int i=0; i<exclusion_list_.size(); i++) {
      float dist = sqrt(pow((std::get<0>(exclusion_list_[i]) - msg.point.x),2) +
        pow((std::get<1>(exclusion_list_[i]) - msg.point.y),2));
      if (dist < std::get<2>(exclusion_list_[i])) {
        return;
      }
    }

    if (accumulate_) {
      for (int i=0; i<latched_filter_list_.size(); i++) {
        cv::Mat dist = (latched_filter_list_[i]->measurementMatrix)*
          (latched_filter_list_[i]->statePost) - meas_state;
        if (abs(cv::sum(dist)[0]) < max_dist_) {
          latched_filter_list_[i]->predict();
          addMeasurement(meas_state, i, true);
          return;
        }
      }
    }

    for (int i=0; i<filter_list_.size(); i++) {
      cv::Mat dist = (filter_list_[i]->measurementMatrix)*(filter_list_[i]->statePost)
        - meas_state;
      if (abs(cv::sum(dist)[0]) < max_dist_) {
        addMeasurement(meas_state, i, false);
        return;
      }
    }
    addFilter(msg);
  }

  /* The process tick for all filters */
  void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    if (filter_list_.size()==0 && latched_filter_list_.size()==0) {
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

    if (accumulate_) {
      publishTop();
    }
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

  bool isGood (std::shared_ptr<cv::KalmanFilter> kf) {
    cv::Mat eigenvalues;
    cv::eigen(kf->errorCovPost, eigenvalues);
    for (int i=0; i<eigenvalues.rows; i++) {
      if (eigenvalues.at<float>(i) < max_pub_cov_) {
        return false;
      }
    }
    return true;
  }

  void checkFilterAges() {
    filter_list_.erase(std::remove_if(filter_list_.begin(), filter_list_.end(),
        std::bind1st(std::mem_fun(&KalmanDetectionFilter::isOld),this)),
        filter_list_.end());
  }

  void processFilters()
  {
    std::vector<int> count;
    for (int i=0; i<filter_list_.size(); i++) {
      cv::Mat eigenvalues;
      cv::eigen(filter_list_[i]->errorCovPost, eigenvalues);
      if (eigenvalues.at<float>(0)+eigenvalues.at<float>(1) < max_pub_cov_) {
        latched_filter_list_.push_back(filter_list_[i]);
        count.push_back(i);
      }
    }

    for (int i=0; i<count.size(); i++) {
      filter_list_.erase(filter_list_.begin()+count[i]);
    }
    count.clear();
  }

  void drawFilterStates() {
    ROS_DEBUG("Number of Filters: %lu", filter_list_.size());
    if (accumulate_) {
      ROS_DEBUG("Number of Latched Filters: %lu", latched_filter_list_.size());
    }
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
    for (auto filter_ptr : latched_filter_list_) {
      cv::Point mean(filter_ptr->statePost.at<float>(0) * px_per_meter,
          filter_ptr->statePost.at<float>(1) * px_per_meter);
      float rad_x = filter_ptr->errorCovPost.at<float>(0,0) * px_per_meter;
      float rad_y = filter_ptr->errorCovPost.at<float>(1,1) * px_per_meter;
      cv::circle(img, mean, 5, cv::Scalar(255,255,0));
      cv::ellipse(img, mean, cv::Size(rad_x, rad_y), 0, 0, 360, cv::Scalar(0,0,255));
    }

    for (int i=0; i<exclusion_list_.size(); i++) {
      cv::circle(img, cv::Point(std::get<0>(exclusion_list_[i])*px_per_meter,
            std::get<1>(exclusion_list_[i])*px_per_meter),
          std::get<2>(exclusion_list_[i])*px_per_meter, cv::Scalar(255,255,255));
    }

    printFilterState();
    cv::imshow("Filter States", img);
    cv::waitKey(10);
  }

  void printFilterState() {
    for (auto filter_ptr : filter_list_) {
      //std::cout << "State: " << filter_ptr->statePost << std::endl;
    }
    for (auto filter_ptr : latched_filter_list_) {
      //std::cout << "Latched State: " << filter_ptr->statePost << std::endl;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_detection_filter");
  KalmanDetectionFilter kdf;
  ros::spin();
}
