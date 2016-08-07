#include <stdio.h>
#include <memory>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <samplereturn_msgs/NamedPoint.h>
#include <samplereturn_msgs/PursuitResult.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>

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
    cv::KalmanFilter filter;
    double hue;
    int16_t filter_id;
    float certainty;
    ColoredKF(cv::KalmanFilter, double, int16_t, float);
};

ColoredKF::ColoredKF (cv::KalmanFilter kf, double h, int16_t id, float cert) {
  filter = kf;
  hue = h;
  filter_id = id;
  certainty = cert;
}

class KalmanDetectionFilter
{
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_cam_info;
  ros::Subscriber sub_ack;
  ros::Subscriber sub_odometry;
  ros::Subscriber sub_exclusion_zone;

  ros::Publisher pub_detection;
  ros::Publisher pub_debug_img;
  ros::Publisher pub_filter_marker_array;
  ros::Publisher pub_frustum_poly;

  std::string detection_topic;
  std::string cam_info_topic;
  std::string ack_topic;
  std::string odometry_topic;
  std::string exclusion_zone_topic;

  std::string filtered_detection_topic;
  std::string debug_img_topic;
  std::string filter_marker_array_topic;
  std::string frustum_poly_topic;

  std::vector<std::shared_ptr<ColoredKF> > filter_list_;
  // Exclusion sites are center,radius,id (x,y,r,id,odometer reading)
  std::vector<std::tuple<float,float,float,int16_t,float> > exclusion_list_;
  // Filters that have ever been published, in case we need their position
  // to create an exclusion zone
  std::map<int16_t, std::shared_ptr<ColoredKF> > published_filters_;

  ros::Time last_time_;
  double max_dist_;
  double max_cov_;
  double max_pub_cov_;
  double max_pub_vel_;

  double pos_exclusion_radius_;
  double neg_exclusion_radius_;

  double process_noise_cov_;
  double measurement_noise_cov_;
  double error_cov_post_;
  double period_;

  int16_t filter_id_count_;
  int16_t current_published_id_;

  int16_t exclusion_count_;

  double PDgO_, PDgo_;
  double PO_init_;
  double certainty_thresh_;

  double last_x_;
  double last_y_;
  bool odometry_received_;
  double odometer_;
  double last_odometry_tick_;
  double odometry_tick_dist_;

  double exclusion_zone_range_;
  bool is_manipulator_;

  double hue_tolerance_;

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
    detection_topic = "named_point";
    ack_topic = "ack";
    odometry_topic = "odometry";
    exclusion_zone_topic = "exclusion_zone";

    filtered_detection_topic = "filtered_point";
    debug_img_topic = "debug_img";
    filter_marker_array_topic = "filter_markers";
    frustum_poly_topic = "frustum_polygon";

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("max_dist", max_dist_, double(0.1));
    private_node_handle_.param("max_cov", max_cov_, double(10.0));
    private_node_handle_.param("max_pub_cov", max_pub_cov_, double(0.1));
    private_node_handle_.param("max_pub_vel", max_pub_vel_, double(0.02));

    private_node_handle_.param("positive_exclusion_radius", pos_exclusion_radius_, double(10.0));
    private_node_handle_.param("negative_exclusion_radius", neg_exclusion_radius_, double(1.5));

    private_node_handle_.param("process_noise_cov", process_noise_cov_, double(0.05));
    private_node_handle_.param("measurement_noise_cov", measurement_noise_cov_, double(0.5));
    private_node_handle_.param("error_cov_post", error_cov_post_, double(0.5));
    private_node_handle_.param("period", period_, double(2));
    private_node_handle_.param("filter_frame_id", _filter_frame_id, std::string("odom"));

    private_node_handle_.param("PDgO", PDgO_, double(0.9));
    private_node_handle_.param("PDgo", PDgo_, double(0.1));
    private_node_handle_.param("certainty_thresh", certainty_thresh_, double(0.9));

    private_node_handle_.param("is_manipulator", is_manipulator_, false);

    sub_cam_info =
      nh.subscribe(cam_info_topic.c_str(), 3, &KalmanDetectionFilter::cameraInfoCallback, this);

    sub_detection =
      nh.subscribe(detection_topic.c_str(), 12, &KalmanDetectionFilter::detectionCallback, this);

    sub_ack = nh.subscribe(ack_topic.c_str(), 3, &KalmanDetectionFilter::ackCallback, this);

    sub_odometry =
      nh.subscribe(odometry_topic.c_str(), 3, &KalmanDetectionFilter::odometryCallback, this);

    sub_exclusion_zone =
      nh.subscribe(exclusion_zone_topic.c_str(), 3, &KalmanDetectionFilter::exclusionZoneCallback, this);

    pub_detection =
      nh.advertise<samplereturn_msgs::NamedPoint>(filtered_detection_topic.c_str(), 3);

    pub_debug_img =
      nh.advertise<sensor_msgs::Image>(debug_img_topic.c_str(), 3);

    pub_filter_marker_array =
      nh.advertise<visualization_msgs::MarkerArray>(filter_marker_array_topic.c_str(), 3);

    pub_frustum_poly =
      nh.advertise<geometry_msgs::PolygonStamped>(frustum_poly_topic.c_str(), 3);

    last_time_.sec = 0.0;
    last_time_.nsec = 0.0;

    filter_id_count_ = 1;
    current_published_id_ = 0;
    exclusion_count_ = 0;

    odometry_received_ = false;
    last_x_ = 0.0;
    last_y_ = 0.0;
    odometer_ = 0.0;
    last_odometry_tick_ = 0.0;
  }

  void exclusionZoneCallback(const std_msgs::Float32 radius)
  {
    // This is for recovery mode. When a message is published to this topic,
    // drop an exclusion zone at current base_link of radius.
    tf::StampedTransform transform;
    listener_.lookupTransform(_filter_frame_id, "base_link", ros::Time(0), transform);
    exclusion_list_.push_back(std::make_tuple(transform.getOrigin().x(),
                                              transform.getOrigin().y(),
                                              radius.data,exclusion_count_,odometer_));
  }

  /* Dynamic reconfigure callback */
  void configCallback(detection_filter::kalman_filter_paramsConfig &config, uint32_t level)
  {
    max_dist_ = config.max_dist;
    max_cov_ = config.max_cov;
    max_pub_cov_ = config.max_pub_cov;
    max_pub_vel_ = config.max_pub_vel;

    process_noise_cov_ = config.process_noise_cov;
    measurement_noise_cov_ = config.measurement_noise_cov;
    error_cov_post_ = config.error_cov_post;
    period_ = config.period;

    PDgO_ = config.PDgO;
    PDgo_ = config.PDgo;
    certainty_thresh_ = config.certainty_thresh;

    odometry_tick_dist_ = config.odometry_tick_dist;

    exclusion_zone_range_ = config.exclusion_zone_range;

    hue_tolerance_ = config.hue_tolerance;

    if(config.clear_filters) {
      //clear all filters
      filter_list_.clear();
      exclusion_list_.clear();
      current_published_id_ = 0;
    }
  }

  /* This is going to inflate the out-of-view filters as the robot moves.
   * This reflects the increasing uncertainty in the position of those
   * hypotheses relative to the robot. Since odometry is absolute,
   * we will differentiate incoming messages and trigger a predict when a
   * distance threshold is reached
   */
  void odometryCallback(const nav_msgs::Odometry& msg)
  {
    if (!odometry_received_) {
      odometry_received_ = true;
      last_x_ = msg.pose.pose.position.x;
      last_y_ = msg.pose.pose.position.y;
    }
    float delta_x = msg.pose.pose.position.x - last_x_;
    float delta_y = msg.pose.pose.position.y - last_y_;
    last_x_ = msg.pose.pose.position.x;
    last_y_= msg.pose.pose.position.y;
    odometer_ += sqrt(pow(delta_x,2)+pow(delta_y,2));
  }

  /* For incoming detections: assign to filter or create new filter
   * For each filter, predict, (update), check for deletion
   * If accumulate_, move to list of points to inspect
   * Publish closest point to inspect
   * When done, delete and add exclusion zone, deleting other filters and inspection points
   */
  void ackCallback(const samplereturn_msgs::PursuitResult& msg)
  {
    auto ackedFilter = published_filters_.find(msg.id);
    if(ackedFilter == published_filters_.end())
    {
        ROS_ERROR_STREAM("Ack for unknown filter id: " << msg.id);
        return;
    }

    ROS_DEBUG_STREAM("Procesing ack for filter id " << msg.id << ".");
    auto activeAckedFilter = std::find_if(filter_list_.begin(), filter_list_.end(),
            [&msg](const std::shared_ptr<ColoredKF> &filter) -> bool
            {
                return msg.id == filter->filter_id;
            });

    if(activeAckedFilter != filter_list_.end())
    {
        ROS_DEBUG_STREAM("Removing filter id " << msg.id << " from active list.");
        auto new_end = std::remove_if(filter_list_.begin(), filter_list_.end(),
                [&](std::shared_ptr<ColoredKF> ckf) {return ckf->filter_id == current_published_id_;});
        filter_list_.erase(new_end, filter_list_.end());
    }
    double x, y, r;
    x = ackedFilter->second->filter.statePost.at<float>(0);
    y = ackedFilter->second->filter.statePost.at<float>(1);
    if (msg.success) {
      r = pos_exclusion_radius_;
    }
    else {
      r = neg_exclusion_radius_;
    }
    exclusion_list_.push_back(std::make_tuple(x,y,r,exclusion_count_,odometer_));
    exclusion_count_ += 1;

    // Clear this Marker from Rviz
    clearMarker(ackedFilter->second);
    published_filters_.erase(msg.id);

    current_published_id_ = 0;
    drawFilterStates();
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
        if (isInView(filter_list_[i]->filter)) {
          filter_list_[i]->filter.predict();
          filter_list_[i]->filter.errorCovPre.copyTo(filter_list_[i]->filter.errorCovPost);;
          double current_PO = filter_list_[i]->certainty;
          filter_list_[i]->certainty = updateProb(current_PO, false, PDgO_, PDgo_);
        }
      }
    }

    checkObservation(msg);
    drawFilterStates();
  }

  /* The process tick for all filters */
  void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    if (filter_list_.size()==0) {
      return;
    }
    if (last_time_ < msg.header.stamp) {
      last_time_ = msg.header.stamp;
      for (int i=0; i<filter_list_.size(); i++) {
        if (isInView(filter_list_[i]->filter) or
            ((odometer_-last_odometry_tick_)>odometry_tick_dist_)) {
          // Manage filters
          filter_list_[i]->filter.predict();
          filter_list_[i]->filter.errorCovPre.copyTo(filter_list_[i]->filter.errorCovPost);;
          double current_PO = filter_list_[i]->certainty;
          filter_list_[i]->certainty = updateProb(current_PO, false, PDgO_, PDgo_);
          // Manage exclusion zones
          auto new_end = std::remove_if(exclusion_list_.begin(),exclusion_list_.end(),
              [this](std::tuple<float,float,float,int16_t,float> zone)
              {return (odometer_ - std::get<4>(zone)) > exclusion_zone_range_;});
          exclusion_list_.erase(new_end, exclusion_list_.end());
        }
      }
    }
    if ((odometer_-last_odometry_tick_) > odometry_tick_dist_) {
      last_odometry_tick_ = odometer_;
    }
    checkFilterAges();
    drawFilterStates();

    publishTop();
  }

  void publishTop() {
    /* If current_published_id_ is nonzero and still viable, keep publishing it
     * Otherwise, publish the nearest viable filter and set it to current */
    ROS_DEBUG("Publish Top");
    if (current_published_id_ != 0) {
      for (auto filter_ptr : filter_list_) {
        if (filter_ptr->filter_id == current_published_id_) {
          if (filter_ptr->certainty > certainty_thresh_ &&
              filter_ptr->filter.errorCovPost.at<float>(0,0) < max_pub_cov_) {
            published_filters_.insert(std::pair<int16_t, std::shared_ptr<ColoredKF> >(filter_ptr->filter_id, filter_ptr));
            samplereturn_msgs::NamedPoint point_msg;
            point_msg.header.frame_id = _filter_frame_id;
            point_msg.header.stamp = ros::Time::now();
            point_msg.point.x = filter_ptr->filter.statePost.at<float>(0);
            point_msg.point.y = filter_ptr->filter.statePost.at<float>(1);
            point_msg.point.z = 0;
            point_msg.filter_id = filter_ptr->filter_id;
            pub_detection.publish(point_msg);
            return;
          }
          else {
            current_published_id_ = 0;
          }
        }
      }
    }

    tf::StampedTransform transform;
    listener_.lookupTransform(_filter_frame_id, "base_link", ros::Time(0), transform);
    float nearest_dist = 10000;
    float dist;
    int nearest_id = 0;
    std::shared_ptr<ColoredKF> nearest_filter;
    /* Walk list to find nearest good filter */
    if (filter_list_.size() > 0) {
      if (current_published_id_ == 0) {
        for (auto filter_ptr : filter_list_) {
          if ((filter_ptr->certainty > certainty_thresh_) &&
              (filter_ptr->filter.errorCovPost.at<float>(0,0) < max_pub_cov_)) {
            dist = sqrt(pow((transform.getOrigin().x()-filter_ptr->filter.statePost.at<float>(0)),2) +
                   pow((transform.getOrigin().y()-filter_ptr->filter.statePost.at<float>(1)),2));
            ROS_DEBUG("Transform X: %f, Transform Y: %f",
                transform.getOrigin().x(),transform.getOrigin().y());
            ROS_DEBUG("Filter Distance: %f",dist);
            if (dist < nearest_dist) {
              nearest_dist = dist;
              nearest_id = filter_ptr->filter_id;
              nearest_filter = filter_ptr;
            }
          }
        }
      }
    }

    if (nearest_id != 0) {
      published_filters_.insert(std::pair<int16_t, std::shared_ptr<ColoredKF> >(nearest_filter->filter_id, nearest_filter));
      current_published_id_ = nearest_id;
      samplereturn_msgs::NamedPoint point_msg;
      point_msg.header.frame_id = _filter_frame_id;
      point_msg.header.stamp = ros::Time::now();
      point_msg.point.x = nearest_filter->filter.statePost.at<float>(0);
      point_msg.point.y = nearest_filter->filter.statePost.at<float>(1);
      point_msg.point.z = 0;
      point_msg.filter_id = nearest_filter->filter_id;
      pub_detection.publish(point_msg);
    }
  }

  void addFilter(const samplereturn_msgs::NamedPoint& msg)
  {
    cv::KalmanFilter KF = cv::KalmanFilter(6,3);
    cv::Mat state(6, 1, CV_32F); /* x, y, z, vx, vy, vz */
    cv::Mat processNoise(6, 1, CV_32F);

    KF.transitionMatrix = (cv::Mat_<float>(6,6) << 1, 0, 0, period_, 0, 0,
                                                    0, 1, 0, 0, period_, 0,
                                                    0, 0, 1, 0, 0, period_,
                                                    0, 0, 0, 1, 0, 0,
                                                    0, 0, 0, 0, 1, 0,
                                                    0, 0, 0, 0, 0, 1);
    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, cv::Scalar(process_noise_cov_));
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(measurement_noise_cov_));
    cv::setIdentity(KF.errorCovPost, cv::Scalar(error_cov_post_));

    KF.statePost.at<float>(0) = msg.point.x;
    KF.statePost.at<float>(1) = msg.point.y;
    KF.statePost.at<float>(2) = msg.point.z;
    KF.statePost.at<float>(3) = 0;
    KF.statePost.at<float>(4) = 0;
    KF.statePost.at<float>(5) = 0;

    KF.predict();
    std::shared_ptr<ColoredKF> CKF (new ColoredKF(KF,msg.hue,filter_id_count_,PO_init_));
    filter_list_.push_back(CKF);
    filter_id_count_++;
    checkObservation(msg);
  }

  void addMeasurement(const cv::Mat meas_state, int filter_index)
  {
    ROS_DEBUG("Adding measurement to filter: %i", filter_index);
    filter_list_[filter_index]->filter.correct(meas_state);
    double current_PO = filter_list_[filter_index]->certainty;
    filter_list_[filter_index]->certainty = updateProb(current_PO, true, PDgO_, PDgo_);
  }

  bool checkColor(double filter_hue, double obs_hue)
  {
    if (is_manipulator_) {
      return true;
    }
    ROS_DEBUG("Color Check: Filter Hue: %f Obs Hue: %f", filter_hue, obs_hue);
    double dist = hueDistance(filter_hue, obs_hue);
    return (dist < hue_tolerance_);
  }

  double hueDistance(double hue, double hue_ref)
  {
    if (abs(hue - hue_ref) <= 90) {
      return abs(hue - hue_ref);
    }
    else {
      return 180 - abs(hue - hue_ref);
    }
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
      double dist = cv::norm(((filter_list_[i]->filter.measurementMatrix)*(filter_list_[i]->filter.statePost)
        - meas_state));
      if ((dist < max_dist_) && checkColor(filter_list_[i]->hue,msg.hue)) {
        ROS_DEBUG("Color Check Passed");
        addMeasurement(meas_state, i);
        filter_list_[i]->hue = msg.hue;
        return;
      }
      else if ((dist < max_dist_) && not checkColor(filter_list_[i]->hue,msg.hue)) {
        ROS_DEBUG("Color Check Failed");
        return;
      }
    }
    addFilter(msg);
  }

  /* This will check if each hypothesis is in view currently */
  bool isInView (cv::KalmanFilter kf) {
    ROS_DEBUG("Is In View Check");
    if (is_manipulator_) {
      return true;
    }
    /* This is in base_link, transform it to odom */
    cv::Mat DSLR_frustum = (cv::Mat_<float>(4,2) <<
        1.0, -3.0, 2.6, -3.0, 2.6, 3.0, 1.0, 3.0);
    cv::Mat DSLR_frustum_odom(4,2,CV_32FC1);
    geometry_msgs::PointStamped temp_msg, temp_msg_odom;
    geometry_msgs::PolygonStamped frustum_poly;
    frustum_poly.header.frame_id = _filter_frame_id;
    frustum_poly.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = "base_link";
    temp_msg.header.stamp = ros::Time(0);
    for (int i=0; i<4; i++) {
      temp_msg.point.x = DSLR_frustum.at<float>(i,0);
      temp_msg.point.y = DSLR_frustum.at<float>(i,1);
      temp_msg.point.z = 0.0;
      try {
        //listener_.waitForTransform("odom", "base_link", temp_msg.header.stamp, ros::Duration(0.2));
        listener_.transformPoint(_filter_frame_id,temp_msg,temp_msg_odom);
      }
      catch (tf::TransformException e) {
        ROS_ERROR_STREAM("Aww shit " << e.what());
        return false;
      }
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
        cv::Point2f(kf.statePost.at<float>(0),kf.statePost.at<float>(1)), false);
    return (retval == 1);
  }

  void checkFilterAges() {
    filter_list_.erase(std::remove_if(filter_list_.begin(), filter_list_.end(),
        std::bind1st(std::mem_fun(&KalmanDetectionFilter::isOld),this)),
        filter_list_.end());
  }

  bool isOld (std::shared_ptr<ColoredKF> ckf) {
    cv::Mat eigenvalues;
    cv::eigen(ckf->filter.errorCovPost, eigenvalues);
    for (int i=0; i<eigenvalues.rows; i++) {
      if (eigenvalues.at<float>(i) > max_cov_) {
        clearMarker(ckf);
        return true;
      }
    }
    if (ckf->certainty < 0.0) {
      clearMarker(ckf);
      return true;
    }
    return false;
  }

  double updateProb(double PO, bool detection, double PDgO, double PDgo) {
    if (detection) {
      // Probability of a detection
      double PD = PDgO * PO + PDgo * (1 - PO);
      // Update
      PO = PDgO * PO / PD;
    }
    else {
      // Probability of no detection
      double Pd = (1 - PDgO) * PO + (1.0 - PDgo) * (1.0 - PO);
      // Update
      PO = (1.0 - PDgO) * PO / Pd;
    }
    return PO;
  }

  void clearMarker(std::shared_ptr<ColoredKF> ckf) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker, text_marker;
    marker.header.frame_id = _filter_frame_id;
    marker.header.stamp = ros::Time::now();
    text_marker.header.frame_id = _filter_frame_id;
    text_marker.header.stamp = ros::Time::now();
    marker.id = ckf->filter_id;
    text_marker.id = 100 + ckf->filter_id;
    marker.action = visualization_msgs::Marker::DELETE;
    text_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(text_marker);
    pub_filter_marker_array.publish(marker_array);
  }

  void drawFilterStates() {
    ROS_DEBUG("Number of Filters: %lu", filter_list_.size());

    visualization_msgs::MarkerArray marker_array;

    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    float px_per_meter = 50.0;
    float offset = 250;
    for (auto filter_ptr : filter_list_){
      cv::Point mean(filter_ptr->filter.statePost.at<float>(0) * px_per_meter,
          filter_ptr->filter.statePost.at<float>(1) * px_per_meter);
      float rad_x = filter_ptr->filter.errorCovPost.at<float>(0,0) * px_per_meter;
      float rad_y = filter_ptr->filter.errorCovPost.at<float>(1,1) * px_per_meter;
      cv::circle(img, mean+cv::Point(0,offset), 5, cv::Scalar(255,0,0));
      cv::ellipse(img, mean+cv::Point(0,offset), cv::Size(rad_x, rad_y), 0, 0, 360, cv::Scalar(0,255,0));

      visualization_msgs::Marker cov, cov_text;
      cov.type = visualization_msgs::Marker::CYLINDER;
      cov_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      /* The radius of the marker is positional covariance, the alpha
       * is the certainty of the observation, squashed from 0-1 to 0.5-1 */
      cov.id = filter_ptr->filter_id;
      cov.header.frame_id = _filter_frame_id;
      cov.header.stamp = ros::Time::now();
      cov_text.id = 100 + filter_ptr->filter_id;
      cov_text.header.frame_id = _filter_frame_id;
      cov_text.header.stamp = ros::Time::now();
      std::stringstream ss;
      ss << "H: " << filter_ptr->hue << "Prob: " << filter_ptr->certainty;
      cov_text.text = ss.str();
      cov_text.color.r = 1.0;
      cov_text.color.g = 1.0;
      cov_text.color.b = 1.0;
      cov_text.color.a = 1.0;
      if (filter_ptr->filter_id == current_published_id_) {
        cov.color.r = 0.0;
        cov.color.g = 1.0;
        cov.color.b = 0.0;
        cov.color.a = (filter_ptr->certainty / 2) + 0.5;
      }
      else {
        cov.color.r = 1.0;
        cov.color.g = 1.0;
        cov.color.b = 1.0;
        cov.color.a = (filter_ptr->certainty / 2) + 0.5;
      }
      cov.pose.position.x = filter_ptr->filter.statePost.at<float>(0);
      cov.pose.position.y = filter_ptr->filter.statePost.at<float>(1);
      cov.pose.position.z = 0.0;
      cov_text.pose.position.x = filter_ptr->filter.statePost.at<float>(0);
      cov_text.pose.position.y = filter_ptr->filter.statePost.at<float>(1);
      cov_text.pose.position.z = 1.0;
      cov.pose.orientation.x = 0;
      cov.pose.orientation.y = 0;
      cov.pose.orientation.z = 0;
      cov.pose.orientation.w = 1;
      cov.scale.x = filter_ptr->filter.errorCovPost.at<float>(0,0);
      cov.scale.y = filter_ptr->filter.errorCovPost.at<float>(1,1);
      cov.scale.z = 1.0;
      cov_text.scale.z = 0.5;
      cov.lifetime = ros::Duration();
      cov_text.lifetime = ros::Duration();
      marker_array.markers.push_back(cov);
      marker_array.markers.push_back(cov_text);
    }

    for (int i=0; i<exclusion_list_.size(); i++) {
      cv::circle(img, cv::Point(std::get<0>(exclusion_list_[i])*px_per_meter,
            std::get<1>(exclusion_list_[i])*px_per_meter),
          std::get<2>(exclusion_list_[i])*px_per_meter, cv::Scalar(255,255,255));
      visualization_msgs::Marker cov;
      cov.type = visualization_msgs::Marker::CYLINDER;
      cov.id = std::get<3>(exclusion_list_[i]);
      cov.ns = "exclusion";
      cov.header.frame_id = _filter_frame_id;
      cov.header.stamp = ros::Time::now();
      cov.color.r = 1.0;
      cov.color.g = 0.0;
      cov.color.b = 0.0;
      cov.color.a = 1.0;
      cov.pose.position.x = std::get<0>(exclusion_list_[i]);
      cov.pose.position.y = std::get<1>(exclusion_list_[i]);
      cov.pose.position.z = 0.0;
      cov.pose.orientation.x = 0;
      cov.pose.orientation.y = 0;
      cov.pose.orientation.z = 0;
      cov.pose.orientation.w = 1;
      cov.scale.x = std::get<2>(exclusion_list_[i])*2;
      cov.scale.y = std::get<2>(exclusion_list_[i])*2;
      cov.scale.z = 0.0;
      cov.lifetime = ros::Duration(2.0);
      marker_array.markers.push_back(cov);
    }

    //printFilterState();
    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",img).toImageMsg();
    pub_debug_img.publish(debug_img_msg);

    pub_filter_marker_array.publish(marker_array);
  }

  void printFilterState() {
    for (auto filter_ptr : filter_list_) {
      std::cout << "State: " << filter_ptr->filter.statePost << std::endl;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_detection_filter");
  KalmanDetectionFilter kdf;
  ros::spin();
}
