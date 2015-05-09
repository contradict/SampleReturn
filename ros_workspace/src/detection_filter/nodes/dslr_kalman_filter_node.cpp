#include <stdio.h>
#include <memory>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
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
    cv::KalmanFilter filter;
    //std::shared_ptr<cv::KalmanFilter> filter;
    std::string color;
    int16_t filter_id;
    float certainty;
    ColoredKF(cv::KalmanFilter, std::string, int16_t, float);
};

ColoredKF::ColoredKF (cv::KalmanFilter kf, std::string c, int16_t id, float cert) {
  filter = kf;
  color = c;
  filter_id = id;
  certainty = cert;
}

class KalmanDetectionFilter
{
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_cam_info;
  ros::Publisher pub_detection;
  ros::Publisher pub_debug_img;
  ros::Publisher pub_filter_marker_array;
  ros::Publisher pub_frustum_poly;

  ros::Subscriber sub_ack;

  std::string cam_info_topic;
  std::string detection_topic;
  std::string filtered_detection_topic;
  std::string ack_topic;
  std::string debug_img_topic;
  std::string filter_marker_array_topic;
  std::string frustum_poly_topic;

  std::vector<std::shared_ptr<ColoredKF> > filter_list_;
  // Exclusion sites are centers and radii (x,y,r)
  std::vector<std::tuple<float,float,float> > exclusion_list_;

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

  int32_t marker_count_;
  int16_t filter_id_count_;
  int16_t current_published_id_;
  std::shared_ptr<ColoredKF> current_published_filter_;

  double certainty_inc_;
  double certainty_dec_;
  double certainty_thresh_;

  tf::TransformListener listener_;

  std::string _filter_frame_id;

  dynamic_reconfigure::Server<detection_filter::kalman_filter_paramsConfig> dr_srv;

  XmlRpc::XmlRpcValue color_transitions_;
  std::map<std::string,std::vector<std::string> > color_transitions_map_;

  public:
  KalmanDetectionFilter()
  {
    dynamic_reconfigure::Server<detection_filter::kalman_filter_paramsConfig>::CallbackType cb;

    cb = boost::bind(&KalmanDetectionFilter::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    cam_info_topic = "camera_info";
    detection_topic = "point";
    filtered_detection_topic = "filtered_point";
    debug_img_topic = "debug_img";
    filter_marker_array_topic = "filter_markers";
    frustum_poly_topic = "frustum_polygon";

    ack_topic = "ack";

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

    private_node_handle_.param("certainty_inc", certainty_inc_, double(1.0));
    private_node_handle_.param("certainty_dec", certainty_dec_, double(0.7));
    private_node_handle_.param("certainty_thresh", certainty_thresh_, double(3.0));

    private_node_handle_.getParam("color_transitions",color_transitions_);
    if (color_transitions_.hasMember(std::string("red"))){
      std::vector<std::string> red_vec;
      for (int i=0; i<color_transitions_[std::string("red")].size(); i++) {
        red_vec.push_back(static_cast<std::string>(color_transitions_[std::string("red")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("red",red_vec));
    }
    if (color_transitions_.hasMember(std::string("orange"))){
      std::vector<std::string> orange_vec;
      for (int i=0; i<color_transitions_[std::string("orange")].size(); i++) {
        orange_vec.push_back(static_cast<std::string>(color_transitions_[std::string("orange")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("orange",orange_vec));
    }
    if (color_transitions_.hasMember(std::string("yellow"))){
      std::vector<std::string> yellow_vec;
      for (int i=0; i<color_transitions_[std::string("yellow")].size(); i++) {
        yellow_vec.push_back(static_cast<std::string>(color_transitions_[std::string("yellow")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("yellow",yellow_vec));
    }
    if (color_transitions_.hasMember(std::string("green"))){
      std::vector<std::string> green_vec;
      for (int i=0; i<color_transitions_[std::string("green")].size(); i++) {
        green_vec.push_back(static_cast<std::string>(color_transitions_[std::string("green")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("green",green_vec));
    }
    if (color_transitions_.hasMember(std::string("blue"))){
      std::vector<std::string> blue_vec;
      for (int i=0; i<color_transitions_[std::string("blue")].size(); i++) {
        blue_vec.push_back(static_cast<std::string>(color_transitions_[std::string("blue")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("blue",blue_vec));
    }
    if (color_transitions_.hasMember(std::string("purple"))){
      std::vector<std::string> purple_vec;
      for (int i=0; i<color_transitions_[std::string("purple")].size(); i++) {
        purple_vec.push_back(static_cast<std::string>(color_transitions_[std::string("purple")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("purple",purple_vec));
    }
    if (color_transitions_.hasMember(std::string("pink"))){
      std::vector<std::string> pink_vec;
      for (int i=0; i<color_transitions_[std::string("pink")].size(); i++) {
        pink_vec.push_back(static_cast<std::string>(color_transitions_[std::string("pink")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("pink",pink_vec));
    }
    if (color_transitions_.hasMember(std::string("brown"))){
      std::vector<std::string> brown_vec;
      for (int i=0; i<color_transitions_[std::string("brown")].size(); i++) {
        brown_vec.push_back(static_cast<std::string>(color_transitions_[std::string("brown")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("brown",brown_vec));
    }
    if (color_transitions_.hasMember(std::string("white"))){
      std::vector<std::string> white_vec;
      for (int i=0; i<color_transitions_[std::string("white")].size(); i++) {
        white_vec.push_back(static_cast<std::string>(color_transitions_[std::string("white")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("white",white_vec));
    }
    if (color_transitions_.hasMember(std::string("gray"))){
      std::vector<std::string> gray_vec;
      for (int i=0; i<color_transitions_[std::string("gray")].size(); i++) {
        gray_vec.push_back(static_cast<std::string>(color_transitions_[std::string("gray")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("gray",gray_vec));
    }
    if (color_transitions_.hasMember(std::string("black"))){
      std::vector<std::string> black_vec;
      for (int i=0; i<color_transitions_[std::string("black")].size(); i++) {
        black_vec.push_back(static_cast<std::string>(color_transitions_[std::string("black")][i]));
      }
      color_transitions_map_.insert(std::pair<std::string,std::vector<std::string> >
          ("black",black_vec));
    }

    sub_cam_info =
      nh.subscribe(cam_info_topic.c_str(), 3, &KalmanDetectionFilter::cameraInfoCallback, this);

    sub_detection =
      nh.subscribe(detection_topic.c_str(), 3, &KalmanDetectionFilter::detectionCallback, this);

    pub_detection =
      nh.advertise<samplereturn_msgs::NamedPoint>(filtered_detection_topic.c_str(), 3);

    pub_debug_img =
      nh.advertise<sensor_msgs::Image>(debug_img_topic.c_str(), 3);

    pub_filter_marker_array =
      nh.advertise<visualization_msgs::MarkerArray>(filter_marker_array_topic.c_str(), 3);

    pub_frustum_poly =
      nh.advertise<geometry_msgs::PolygonStamped>(frustum_poly_topic.c_str(), 3);

    sub_ack = nh.subscribe(ack_topic.c_str(), 3, &KalmanDetectionFilter::ackCallback, this);

    last_time_.sec = 0.0;
    last_time_.nsec = 0.0;

    marker_count_ = 0;
    filter_id_count_ = 1;
    current_published_id_ = 0;
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

    certainty_inc_ = config.certainty_inc;
    certainty_dec_ = config.certainty_dec;
    certainty_thresh_ = config.certainty_thresh;

    if(config.clear_filters) {
      //clear all filters
      filter_list_.clear();
      current_published_id_ = 0;
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
    x = current_published_filter_->filter.statePost.at<float>(0);
    y = current_published_filter_->filter.statePost.at<float>(1);
    if (msg.success) {
      r = pos_exclusion_radius_;
    }
    else {
      r = neg_exclusion_radius_;
    }
    exclusion_list_.push_back(std::make_tuple(x,y,r));

    auto new_end = std::remove_if(filter_list_.begin(), filter_list_.end(),
        [&](std::shared_ptr<ColoredKF> ckf) {return ckf->filter_id == current_published_id_;});
    filter_list_.erase(new_end, filter_list_.end());
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
        filter_list_[i]->filter.predict();
      }
    }

    checkObservation(msg);
    drawFilterStates();
  }

  void publishTop() {
    ROS_DEBUG("Publish Top");
    tf::StampedTransform transform;
    listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);
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
    std::shared_ptr<ColoredKF> CKF (new ColoredKF(KF,msg.name,filter_id_count_,0));
    filter_list_.push_back(CKF);
    filter_id_count_++;
    checkObservation(msg);
  }

  void addMeasurement(const cv::Mat meas_state, int filter_index)
  {
    ROS_DEBUG("Adding measurement to filter: %i", filter_index);
    filter_list_[filter_index]->filter.correct(meas_state);
    filter_list_[filter_index]->certainty += (certainty_inc_+certainty_dec_);
  }

  bool checkColor(std::string filter_color, std::string obs_color)
  {
    std::vector<std::string>::iterator color_it;
    color_it = std::find(color_transitions_map_[filter_color].begin(),
                          color_transitions_map_[filter_color].end(),
                          obs_color);
    ROS_DEBUG("Color Check: Filter Color:%s Obs Color:%s",filter_color.c_str(),obs_color.c_str());
    return (color_it != color_transitions_map_[filter_color].end());
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
      if ((dist < max_dist_) && checkColor(filter_list_[i]->color,msg.name)) {
        ROS_DEBUG("Color Check Passed");
        addMeasurement(meas_state, i);
        filter_list_[i]->color = msg.name;
        return;
      }
      else if ((dist < max_dist_) && not checkColor(filter_list_[i]->color,msg.name)) {
        ROS_DEBUG("Color Check Failed");
        return;
      }
    }
    addFilter(msg);
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
        if (isInView(filter_list_[i]->filter)) {
          filter_list_[i]->filter.predict();
          filter_list_[i]->filter.errorCovPre.copyTo(filter_list_[i]->filter.errorCovPost);;
          filter_list_[i]->certainty -= certainty_dec_;
        }
      }
    }
    checkFilterAges();
    drawFilterStates();

    publishTop();
  }

  /* This will check if each hypothesis is in view currently */
  bool isInView (cv::KalmanFilter kf) {
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
    temp_msg.header.stamp = ros::Time(0);
    for (int i=0; i<4; i++) {
      temp_msg.point.x = DSLR_frustum.at<float>(i,0);
      temp_msg.point.y = DSLR_frustum.at<float>(i,1);
      temp_msg.point.z = 0.0;
      try {
        //listener_.waitForTransform("odom", "base_link", temp_msg.header.stamp, ros::Duration(0.2));
        listener_.transformPoint("odom",temp_msg,temp_msg_odom);
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

  bool isOld (std::shared_ptr<ColoredKF> ckf) {
    cv::Mat eigenvalues;
    cv::eigen(ckf->filter.errorCovPost, eigenvalues);
    for (int i=0; i<eigenvalues.rows; i++) {
      if (eigenvalues.at<float>(i) > max_cov_) {
        clearMarker(ckf);
        return true;
      }
    }
    if (ckf->certainty < -1.0) {
      clearMarker(ckf);
      return true;
    }
    return false;
  }

  void clearMarker(std::shared_ptr<ColoredKF> ckf) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = ckf->filter_id;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(marker);
    pub_filter_marker_array.publish(marker_array);
  }

  void checkFilterAges() {
    filter_list_.erase(std::remove_if(filter_list_.begin(), filter_list_.end(),
        std::bind1st(std::mem_fun(&KalmanDetectionFilter::isOld),this)),
        filter_list_.end());
  }

  void drawFilterStates() {
    ROS_DEBUG("Number of Filters: %lu", filter_list_.size());

    visualization_msgs::MarkerArray marker_array;

    marker_count_ = 0;

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

      visualization_msgs::Marker cov;
      cov.type = visualization_msgs::Marker::CYLINDER;
      /* The radius of the marker is positional covariance, the height
       * is the certainty of the observation */
      cov.id = filter_ptr->filter_id;
      cov.header.frame_id = "odom";
      cov.header.stamp = ros::Time::now();
      if (filter_ptr->filter_id == current_published_id_) {
        cov.color.r = 0.0;
        cov.color.g = 1.0;
        cov.color.b = 0.0;
        cov.color.a = 1.0;
      }
      else {
        cov.color.r = 1.0;
        cov.color.g = 1.0;
        cov.color.b = 1.0;
        cov.color.a = 0.5;
      }
      cov.pose.position.x = filter_ptr->filter.statePost.at<float>(0);
      cov.pose.position.y = filter_ptr->filter.statePost.at<float>(1);
      cov.pose.position.z = 0.0;
      cov.pose.orientation.x = 0;
      cov.pose.orientation.y = 0;
      cov.pose.orientation.z = 0;
      cov.pose.orientation.w = 1;
      cov.scale.x = filter_ptr->filter.errorCovPost.at<float>(0,0);
      cov.scale.y = filter_ptr->filter.errorCovPost.at<float>(1,1);
      cov.scale.z = filter_ptr->certainty;
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
