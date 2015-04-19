#include <stdio.h>
#include <memory>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <samplereturn_msgs/NamedPoint.h>
#include <samplereturn_msgs/PursuitResult.h>
#include <geometry_msgs/PolygonStamped.h>

#include <dynamic_reconfigure/server.h>
#include <detection_filter/kalman_filter_paramsConfig.h>

/* This is going to subscribe to a detection channel, maintain some number
 * of Kalman filters for hypothesis, and publish confirmed detections
 * when the covariance and velocity of a filter fall below a threshold.
 * It will also age out filters that have too large a convariance.
 */

class ColoredKF
{
  public:
    std::shared_ptr<cv::KalmanFilter> filter;
    std::string color;
    ColoredKF(std::shared_ptr<cv::KalmanFilter>, std::string);
};

ColoredKF::ColoredKF (std::shared_ptr<cv::KalmanFilter> kf, std::string c) {
  filter = kf;
  color = c;
}

class KalmanDetectionFilter
{
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_img_detection;
  ros::Subscriber sub_cam_info;
  ros::Publisher pub_detection;
  ros::Publisher pub_img_detection;
  ros::Publisher pub_debug_img;
  ros::Publisher pub_filter_marker_array;
  ros::Publisher pub_frustum_poly;

  ros::Subscriber sub_ack;

  std::string cam_info_topic;
  std::string detection_topic;
  std::string img_detection_topic;
  std::string filtered_detection_topic;
  std::string filtered_img_detection_topic;
  std::string ack_topic;
  std::string debug_img_topic;
  std::string filter_marker_array_topic;
  std::string frustum_poly_topic;

  std::vector<ColoredKF> filter_list_;
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

  double process_noise_cov_;
  double measurement_noise_cov_;
  double error_cov_post_;
  double period_;

  int32_t marker_count_;

  image_geometry::PinholeCameraModel cam_model_;
  tf::TransformListener listener_;

  std::string _filter_frame_id;

  dynamic_reconfigure::Server<detection_filter::kalman_filter_paramsConfig> dr_srv;

  public:
  KalmanDetectionFilter()
  {
    dynamic_reconfigure::Server<detection_filter::kalman_filter_paramsConfig>::CallbackType cb;

    cb = boost::bind(&KalmanDetectionFilter::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    cam_info_topic = "camera_info";
    //cam_info_topic = "/cameras/manipulator/left/camera_info";
    detection_topic = "point";
    img_detection_topic = "img_point";
    filtered_detection_topic = "filtered_point";
    filtered_img_detection_topic = "filtered_img_point";
    debug_img_topic = "debug_img";
    filter_marker_array_topic = "filter_markers";
    frustum_poly_topic = "frustum_polygon";

    ack_topic = "ack";

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("max_dist", max_dist_, double(0.1));
    private_node_handle_.param("max_cov", max_cov_, double(10.0));
    private_node_handle_.param("max_pub_cov", max_pub_cov_, double(0.1));
    private_node_handle_.param("max_pub_vel", max_pub_vel_, double(0.02));

    private_node_handle_.param("accumulate", accumulate_, false);
    private_node_handle_.param("positive_exclusion_radius", pos_exclusion_radius_, double(10.0));
    private_node_handle_.param("negative_exclusion_radius", neg_exclusion_radius_, double(1.5));

    private_node_handle_.param("process_noise_cov", process_noise_cov_, double(0.05));
    private_node_handle_.param("measurement_noise_cov", measurement_noise_cov_, double(0.5));
    private_node_handle_.param("error_cov_post", error_cov_post_, double(0.5));
    private_node_handle_.param("period", period_, double(2));
    private_node_handle_.param("filter_frame_id", _filter_frame_id, std::string("odom"));

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

    pub_debug_img =
      nh.advertise<sensor_msgs::Image>(debug_img_topic.c_str(), 3);

    pub_filter_marker_array =
      nh.advertise<visualization_msgs::MarkerArray>(filter_marker_array_topic.c_str(), 3);

    pub_frustum_poly =
      nh.advertise<geometry_msgs::PolygonStamped>(frustum_poly_topic.c_str(), 3);

    if(accumulate_) {
      sub_ack = nh.subscribe(ack_topic.c_str(), 3, &KalmanDetectionFilter::ackCallback, this);
    }

    last_time_.sec = 0.0;
    last_time_.nsec = 0.0;

    marker_count_ = 0;
  }

  /* Dynamic reconfigure callback */
  void configCallback(detection_filter::kalman_filter_paramsConfig &config, uint32_t level)
  {
    ROS_INFO("configCallback");
    max_dist_ = config.max_dist;
    max_cov_ = config.max_cov;
    max_pub_cov_ = config.max_pub_cov;
    max_pub_vel_ = config.max_pub_vel;

    process_noise_cov_ = config.process_noise_cov;
    measurement_noise_cov_ = config.measurement_noise_cov;
    error_cov_post_ = config.error_cov_post;
    period_ = config.period;

    if(config.clear_filters) {
      //clear all filters
      filter_list_.clear();
    }
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
    x = filter_list_[0].filter->statePost.at<float>(0);
    y = filter_list_[0].filter->statePost.at<float>(1);
    if (msg.success) {
      r = pos_exclusion_radius_;
    }
    else {
      r = neg_exclusion_radius_;
    }
    exclusion_list_.push_back(std::make_tuple(x,y,r));

    filter_list_.erase(filter_list_.begin());
  }

  void imgDetectionCallback(const samplereturn_msgs::NamedPoint& msg)
  {
    last_img_point_msg_ = msg;
    got_img_point_ = true;
  }

  void detectionCallback(const samplereturn_msgs::NamedPoint& msg)
  {
    if (filter_list_.size() == 0) {
      addFilter(msg);
      return;
    }
    if (last_time_ < msg.header.stamp) {
      last_time_ = msg.header.stamp;
      for (int i=0; i<filter_list_.size(); i++) {
        filter_list_[i].filter->predict();
      }
    }

    checkObservation(msg);

    publishFilteredDetections(msg);
  }

  void publishTop() {
    if (filter_list_.size() > 0) {
      samplereturn_msgs::NamedPoint point_msg;
      point_msg.header.frame_id = _filter_frame_id;
      point_msg.header.stamp = ros::Time::now();
      point_msg.point.x = filter_list_[0].filter->statePost.at<float>(0);
      point_msg.point.y = filter_list_[0].filter->statePost.at<float>(1);
      point_msg.point.z = 0;
      pub_detection.publish(point_msg);

      if (cam_model_.initialized()) {
        cv::Point3d xyz_point;
        xyz_point.x = double(filter_list_[0].filter->statePost.at<float>(0));
        xyz_point.y = double(filter_list_[0].filter->statePost.at<float>(1));
        xyz_point.z = double(filter_list_[0].filter->statePost.at<float>(2));
        cv::Point2d uv_point = cam_model_.project3dToPixel(xyz_point);
        samplereturn_msgs::NamedPoint img_point_msg;
        img_point_msg.header.frame_id = "";
        img_point_msg.header.stamp = ros::Time::now();
        img_point_msg.point.x = uv_point.x;
        img_point_msg.point.y = uv_point.y;
        img_point_msg.point.z = 0;
        pub_img_detection.publish(img_point_msg);
      }
    }
  }

  void publishFilteredDetections(const samplereturn_msgs::NamedPoint& msg) {
    for (auto filter_ptr : filter_list_) {
      cv::Mat eigenvalues;
      cv::eigen(filter_ptr.filter->errorCovPost, eigenvalues);
      for (int i=0; i<eigenvalues.rows; i++) {
        if (eigenvalues.at<float>(i) > max_pub_cov_) {
          break;
        }
      }
      if (filter_ptr.filter->statePost.at<float>(3) < max_pub_vel_ ||
          filter_ptr.filter->statePost.at<float>(4) < max_pub_vel_) {
        samplereturn_msgs::NamedPoint point_msg;
        point_msg.header.frame_id = _filter_frame_id;
        point_msg.header.stamp = ros::Time::now();
        point_msg.grip_angle = msg.grip_angle;
        point_msg.sample_id = msg.sample_id;
        point_msg.point.x = filter_ptr.filter->statePost.at<float>(0);
        point_msg.point.y = filter_ptr.filter->statePost.at<float>(1);
        point_msg.point.z = 0;
        pub_detection.publish(point_msg);
      }
      if (cam_model_.initialized()) {
        cv::Point3d xyz_point;
        xyz_point.x = double(filter_list_[0].filter->statePost.at<float>(0));
        xyz_point.y = double(filter_list_[0].filter->statePost.at<float>(1));
        xyz_point.z = double(filter_list_[0].filter->statePost.at<float>(2));
        geometry_msgs::PointStamped odom_point;
        odom_point.header = msg.header;
        odom_point.point.x = xyz_point.x;
        odom_point.point.y = xyz_point.y;
        odom_point.point.z = xyz_point.z;
        geometry_msgs::PointStamped temp_point;
        try {
          listener_.waitForTransform("manipulator_left_camera", msg.header.frame_id, msg.header.stamp, ros::Duration(1.0));
        }
        catch (tf::TransformException e) {
          ROS_ERROR_STREAM("Aww shit " << e.what());
        }
        listener_.transformPoint("manipulator_left_camera", odom_point, temp_point);
        cv::Point3d cam_xyz_point;
        cam_xyz_point.x = temp_point.point.x;
        cam_xyz_point.y = temp_point.point.y;
        cam_xyz_point.z = temp_point.point.z;
        cam_xyz_point.x /= cam_xyz_point.z;
        cam_xyz_point.y /= cam_xyz_point.z;
        cam_xyz_point.z /= cam_xyz_point.z;
        cv::Point2d uv_point = cam_model_.project3dToPixel(cam_xyz_point);
        samplereturn_msgs::NamedPoint img_point_msg;
        img_point_msg.header.frame_id = "";
        img_point_msg.header.stamp = ros::Time::now();
        img_point_msg.point.x = uv_point.x;
        img_point_msg.point.y = uv_point.y;
        img_point_msg.point.z = 0;
        pub_img_detection.publish(img_point_msg);
      }
    }
  }

  void addFilter(const samplereturn_msgs::NamedPoint& msg)
  {
    std::shared_ptr<cv::KalmanFilter> KF (new cv::KalmanFilter(6,3));
    cv::Mat state(6, 1, CV_32F); /* x, y, z, vx, vy, vz */
    cv::Mat processNoise(6, 1, CV_32F);

    KF->transitionMatrix = (cv::Mat_<float>(6,6) << 1, 0, 0, period_, 0, 0,
                                                    0, 1, 0, 0, period_, 0,
                                                    0, 0, 1, 0, 0, period_,
                                                    0, 0, 0, 1, 0, 0,
                                                    0, 0, 0, 0, 1, 0,
                                                    0, 0, 0, 0, 0, 1);
    cv::setIdentity(KF->measurementMatrix);
    cv::setIdentity(KF->processNoiseCov, cv::Scalar(process_noise_cov_));
    cv::setIdentity(KF->measurementNoiseCov, cv::Scalar(measurement_noise_cov_));
    cv::setIdentity(KF->errorCovPost, cv::Scalar(error_cov_post_));

    KF->statePost.at<float>(0) = msg.point.x;
    KF->statePost.at<float>(1) = msg.point.y;
    KF->statePost.at<float>(2) = msg.point.z;
    KF->statePost.at<float>(3) = 0;
    KF->statePost.at<float>(4) = 0;
    KF->statePost.at<float>(5) = 0;

    KF->predict();
    filter_list_.push_back(ColoredKF(KF,"red"));
    checkObservation(msg);
  }

  void addMeasurement(const cv::Mat meas_state, int filter_index)
  {
    ROS_DEBUG("Adding measurement to filter: %i", filter_index);
    filter_list_[filter_index].filter->correct(meas_state);
  }

  void checkObservation(const samplereturn_msgs::NamedPoint& msg)
  {
    cv::Mat meas_state(3, 1, CV_32F);
    meas_state.at<float>(0) = msg.point.x;
    meas_state.at<float>(1) = msg.point.y;
    meas_state.at<float>(2) = msg.point.z;

    for (int i=0; i<exclusion_list_.size(); i++) {
      float dist = sqrt(pow((std::get<0>(exclusion_list_[i]) - msg.point.x),2) +
        pow((std::get<1>(exclusion_list_[i]) - msg.point.y),2));
      if (dist < std::get<2>(exclusion_list_[i])) {
        return;
      }
    }

    for (int i=0; i<filter_list_.size(); i++) {
      cv::Mat dist = (filter_list_[i].filter->measurementMatrix)*(filter_list_[i].filter->statePost)
        - meas_state;
      if (abs(cv::sum(dist)[0]) < max_dist_) {
        addMeasurement(meas_state, i);
        return;
      }
    }
    addFilter(msg);
  }

  /* The process tick for all filters */
  void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    if (!cam_model_.initialized()) {
      cam_model_.fromCameraInfo(msg);
    }
    if (filter_list_.size()==0) {
      return;
    }
    if (last_time_ < msg.header.stamp) {
      last_time_ = msg.header.stamp;
      for (int i=0; i<filter_list_.size(); i++) {
        if (isInView(filter_list_[i].filter)) {
          filter_list_[i].filter->predict();
          filter_list_[i].filter->errorCovPre.copyTo(filter_list_[i].filter->errorCovPost);;
        }
      }
    }
    checkFilterAges();
    drawFilterStates();

    if (accumulate_) {
      publishTop();
    }
  }

  /* This will check if each hypothesis is in view currently */
  bool isInView (std::shared_ptr<cv::KalmanFilter> kf) {
    ROS_DEBUG("Is In View Check");
    /* This is in base_link, transform it to odom */
    cv::Mat DSLR_frustum = (cv::Mat_<float>(4,2) <<
        1.75, -1.21, 21.75, -13.21, 21.75, 13.21, 1.75, 1.21);
    cv::Mat DSLR_frustum_odom(4,2,CV_32FC1);
    geometry_msgs::PointStamped temp_msg, temp_msg_odom;
    geometry_msgs::PolygonStamped frustum_poly;
    frustum_poly.header.frame_id = "odom";
    frustum_poly.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = "base_link";
    temp_msg.header.stamp = ros::Time::now();
    for (int i=0; i<4; i++) {
      temp_msg.point.x = DSLR_frustum.at<float>(i,0);
      temp_msg.point.y = DSLR_frustum.at<float>(i,1);
      temp_msg.point.z = 0.0;
      try {
        listener_.waitForTransform("odom", "base_link", temp_msg.header.stamp, ros::Duration(0.2));
      }
      catch (tf::TransformException e) {
        ROS_ERROR_STREAM("Aww shit " << e.what());
      }
      listener_.transformPoint("odom",temp_msg,temp_msg_odom);
      DSLR_frustum_odom.at<float>(i,0) = temp_msg_odom.point.x;
      DSLR_frustum_odom.at<float>(i,1) = temp_msg_odom.point.y;
      geometry_msgs::Point32 temp_point;
      temp_point.x = temp_msg_odom.point.x;
      temp_point.y = temp_msg_odom.point.y;
      temp_point.z = 0.0;
      frustum_poly.polygon.points.push_back(temp_point);
    }
    pub_frustum_poly.publish(frustum_poly);
    double retval = cv::pointPolygonTest(DSLR_frustum_odom,
        cv::Point2f(kf->statePost.at<float>(0),kf->statePost.at<float>(1)), false);
    return (retval == 1);
  }

  bool isOld (ColoredKF ckf) {
    cv::Mat eigenvalues;
    cv::eigen(ckf.filter->errorCovPost, eigenvalues);
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

  void drawFilterStates() {
    ROS_DEBUG("Number of Filters: %lu", filter_list_.size());

    visualization_msgs::MarkerArray marker_array;

    for (int i=0; i<marker_count_; i++) {
      visualization_msgs::Marker clear_marker;
      clear_marker.header.frame_id = "/map";
      clear_marker.header.stamp = ros::Time::now();
      clear_marker.id = i;
      clear_marker.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(clear_marker);
    }
    pub_filter_marker_array.publish(marker_array);
    marker_array.markers.clear();

    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    float px_per_meter = 50.0;
    float offset = 250;
    for (auto filter_ptr : filter_list_){
      cv::Point mean(filter_ptr.filter->statePost.at<float>(0) * px_per_meter,
          filter_ptr.filter->statePost.at<float>(1) * px_per_meter);
      float rad_x = filter_ptr.filter->errorCovPost.at<float>(0,0) * px_per_meter;
      float rad_y = filter_ptr.filter->errorCovPost.at<float>(1,1) * px_per_meter;
      cv::circle(img, mean+cv::Point(0,offset), 5, cv::Scalar(255,0,0));
      cv::ellipse(img, mean+cv::Point(0,offset), cv::Size(rad_x, rad_y), 0, 0, 360, cv::Scalar(0,255,0));

      visualization_msgs::Marker cov;
      cov.type = visualization_msgs::Marker::SPHERE;
      cov.id = marker_count_;
      cov.header.frame_id = "odom";
      cov.header.stamp = ros::Time::now();
      cov.color.r = 1.0;
      cov.color.g = 1.0;
      cov.color.b = 1.0;
      cov.color.a = 0.5;
      cov.pose.position.x = filter_ptr.filter->statePost.at<float>(0);
      cov.pose.position.y = filter_ptr.filter->statePost.at<float>(1);
      cov.pose.position.z = 0.0;
      cov.pose.orientation.x = 0;
      cov.pose.orientation.y = 0;
      cov.pose.orientation.z = 0;
      cov.pose.orientation.w = 1;
      cov.scale.x = filter_ptr.filter->errorCovPost.at<float>(0,0);
      cov.scale.y = filter_ptr.filter->errorCovPost.at<float>(1,1);
      cov.scale.z = 0.01;
      cov.lifetime = ros::Duration();
      marker_array.markers.push_back(cov);
      marker_count_ += 1;
    }

    for (int i=0; i<exclusion_list_.size(); i++) {
      cv::circle(img, cv::Point(std::get<0>(exclusion_list_[i])*px_per_meter,
            std::get<1>(exclusion_list_[i])*px_per_meter),
          std::get<2>(exclusion_list_[i])*px_per_meter, cv::Scalar(255,255,255));
    }

    //printFilterState();
    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",img).toImageMsg();
    pub_debug_img.publish(debug_img_msg);

    pub_filter_marker_array.publish(marker_array);
  }

  void printFilterState() {
    for (auto filter_ptr : filter_list_) {
      std::cout << "State: " << filter_ptr.filter->statePost << std::endl;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_detection_filter");
  KalmanDetectionFilter kdf;
  ros::spin();
}
