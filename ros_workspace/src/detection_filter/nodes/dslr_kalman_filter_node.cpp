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
#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <samplereturn_msgs/NamedPointArray.h>
#include <samplereturn_msgs/PursuitResult.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>
#include <detection_filter/kalman_filter_paramsConfig.h>
#include <samplereturn/colormodel.h>
#include <detection_filter/coloredkf.hpp>

/* This is going to subscribe to a detection channel, maintain some number
 * of Kalman filters for hypothesis, and publish confirmed detections
 * when the covariance and velocity of a filter fall below a threshold.
 * It will also age out filters that have too large a convariance.
 */

namespace detection_filter {
class KalmanDetectionFilter
{
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_ack;
  ros::Subscriber sub_odometry;
  ros::Subscriber sub_exclusion_zone;
  ros::Subscriber sub_pause_state;
  ros::Subscriber sub_view_frusta;

  ros::Publisher pub_detection;
  ros::Publisher pub_debug_img;
  ros::Publisher pub_filter_marker_array;
  ros::Publisher pub_frustum_poly;

  std::vector<std::shared_ptr<ColoredKF> > filter_list_;
  // Exclusion sites are center,radius,id (x,y,r,id,odometer reading)
  std::vector<std::tuple<float,float,float,int16_t,float> > exclusion_list_;
  // Filters that have ever been published, in case we need their position
  // to create an exclusion zone
  std::map<int16_t, std::shared_ptr<ColoredKF> > published_filters_;

  kalman_filter_paramsConfig config_;

  int16_t filter_id_count_;
  int16_t current_published_id_;

  int16_t exclusion_count_;

  double last_x_;
  double last_y_;
  bool odometry_received_;
  double odometer_;
  double last_odometry_tick_;

  bool is_paused_;

  std::map<std::string, cv::Mat> search_frusta_;

  tf::TransformListener listener_;

  std::string _filter_frame_id;

  dynamic_reconfigure::Server<detection_filter::kalman_filter_paramsConfig> dr_srv;

  public:
  KalmanDetectionFilter()
  {
    dynamic_reconfigure::Server<detection_filter::kalman_filter_paramsConfig>::CallbackType cb;

    cb = boost::bind(&KalmanDetectionFilter::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");

    private_node_handle_.param("filter_frame_id", _filter_frame_id, std::string("odom"));

    sub_detection =
      nh.subscribe("named_point", 12, &KalmanDetectionFilter::detectionCallback, this);

    sub_ack = nh.subscribe("ack", 3, &KalmanDetectionFilter::ackCallback, this);

    sub_odometry =
      nh.subscribe("odometry", 3, &KalmanDetectionFilter::odometryCallback, this);

    sub_exclusion_zone =
      nh.subscribe("exclusion_zone", 3, &KalmanDetectionFilter::exclusionZoneCallback, this);

    sub_pause_state =
      nh.subscribe("pause_state", 1, &KalmanDetectionFilter::pauseStateCallback, this);

    sub_view_frusta =
      nh.subscribe("view_frusta", 1, &KalmanDetectionFilter::viewFrustaCallback, this);

    pub_detection =
      nh.advertise<samplereturn_msgs::NamedPoint>("filtered_point", 3);

    pub_debug_img =
      nh.advertise<sensor_msgs::Image>("debug_img", 3);

    pub_filter_marker_array =
      nh.advertise<visualization_msgs::MarkerArray>("filtered_markers", 3);

    pub_frustum_poly =
      nh.advertise<geometry_msgs::PolygonStamped>("frustum_polygon", 3);

    filter_id_count_ = 1;
    current_published_id_ = 0;
    exclusion_count_ = 0;
    is_paused_ = false;

    odometry_received_ = false;
    last_x_ = 0.0;
    last_y_ = 0.0;
    odometer_ = 0.0;
    last_odometry_tick_ = 0.0;
  }

  void viewFrustaCallback(const geometry_msgs::PolygonStamped view_frustum)
  {
    // Take in a frustum message, write into matching field in map from frame_id
    std::vector<cv::Point2f> cv_points;
    std::transform(view_frustum.polygon.points.begin(), view_frustum.polygon.points.end(),
        cv_points.begin(),
        [](geometry_msgs::Point32 pt) -> cv::Point2f
        {
          return cv::Point2f(pt.x, pt.y);
        });
    cv::Mat_<float> mat(4, 2, (float*)cv_points.data());
    search_frusta_[view_frustum.header.frame_id] = mat.clone();
  }

  void pauseStateCallback(const std_msgs::Bool pause_state)
  {
    is_paused_ = pause_state.data;
    if (is_paused_) {
      ROS_DEBUG("Search Kalman Filter Paused");
    }
    else {
      ROS_DEBUG("Search Kalman Filter Running");
    }
  }

  void exclusionZoneCallback(const std_msgs::Float32 radius)
  {
    // This is for recovery mode. When a message is published to this topic,
    // drop an exclusion zone at current base_link of radius.
    tf::StampedTransform transform;
    try
    {
        listener_.lookupTransform(_filter_frame_id, "base_link", ros::Time(0), transform);
    }
    catch(tf::TransformException e)
    {
        ROS_INFO_STREAM("Could not transform to " << _filter_frame_id << " : " << e.what());
        return;
    }
    exclusion_list_.push_back(std::make_tuple(transform.getOrigin().x(),
                                              transform.getOrigin().y(),
                                              radius.data,exclusion_count_,odometer_));
  }

  /* Dynamic reconfigure callback */
  void configCallback(detection_filter::kalman_filter_paramsConfig &config, uint32_t level)
  {
      (void)level;
      config_ = config;

    if(config.clear_filters) {
      //clear all filters
      filter_list_.clear();
      exclusion_list_.clear();
      current_published_id_ = 0;
      config.clear_filters = false;
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
    x = ackedFilter->second->statePost.at<float>(0);
    y = ackedFilter->second->statePost.at<float>(1);
    if (msg.success) {
      r = config_.positive_exclusion_radius;
    }
    else {
      r = config_.negative_exclusion_radius;
    }
    exclusion_list_.push_back(std::make_tuple(x,y,r,exclusion_count_,odometer_));
    exclusion_count_ += 1;

    // Clear this Marker from Rviz
    clearMarker(ackedFilter->second);
    published_filters_.erase(msg.id);

    current_published_id_ = 0;
    drawFilterStates();
  }

  void detectionCallback(const samplereturn_msgs::NamedPointArrayConstPtr& msg)
  {
    ROS_DEBUG("Detection Callback");
    // Draw in Rviz
    drawFilterStates();
    // Publish top filter for pursuit
    publishTop();
    // no filter changes if paused
    if (is_paused_) {
      return;
    }

    // write down all filters to be sure they get some update
    std::vector<std::shared_ptr<ColoredKF> > not_updated_filter_list;
    for (auto ckf : filter_list_) {
      ROS_DEBUG("Checking Filter to NUF");
      ROS_DEBUG("%s",ckf->frame_id.c_str());
      ROS_DEBUG("%s",msg->header.frame_id.c_str());
      if (ckf->frame_id.compare(msg->header.frame_id) == 0) {
        ROS_DEBUG("Adding Filter to NUF");
        not_updated_filter_list.push_back(ckf);
      }
    }

    for(const auto& np : msg->points)
    {

        int filter_id = checkObservation(np);

        // Iterate over appropriate NUF list, remove the matching filter
        std::vector<std::shared_ptr<ColoredKF> >::iterator iter =
            not_updated_filter_list.begin();
        while ( iter != not_updated_filter_list.end() ) {
            if ((*iter)->filter_id == filter_id) {
                iter = not_updated_filter_list.erase(iter);
                break;
            }
            ++iter;
        }
    }

    // Get filters in msg that weren't updated, decrement them
    for (auto ckf : not_updated_filter_list)
    {
      ROS_DEBUG("Checking NUF");
      if (isInView(ckf) or (odometer_-last_odometry_tick_)>config_.odometry_tick_dist) {
        ckf->predict();
        ckf->measure(config_.PDgO, config_.PDgo);
        ROS_DEBUG("Negative observation Updated Prob: %f",ckf->certainty);
      }
    }

    // Toss too uncertain filters
    checkFilterAges();

    // Manage exclusion zones
    auto new_end = std::remove_if(exclusion_list_.begin(),exclusion_list_.end(),
        [this](std::tuple<float,float,float,int16_t,float> zone)
        {return (odometer_ - std::get<4>(zone)) > config_.exclusion_zone_range;});
    exclusion_list_.erase(new_end, exclusion_list_.end());

    // Update odometer tick
    if ((odometer_-last_odometry_tick_) > config_.odometry_tick_dist) {
      last_odometry_tick_ = odometer_;
    }

  }

  void publishTop() {
    /* If current_published_id_ is nonzero and still viable, keep publishing it
     * Otherwise, publish the nearest viable filter and set it to current */
    ROS_DEBUG("Publish Top");
    if (current_published_id_ != 0) {
      for (auto filter_ptr : filter_list_) {
        if (filter_ptr->filter_id == current_published_id_) {
          if (filter_ptr->certainty > config_.pub_certainty_thresh &&
              filter_ptr->errorCovPost.at<float>(0,0) < config_.max_pub_cov) {
            published_filters_.insert(std::pair<int16_t, std::shared_ptr<ColoredKF> >(filter_ptr->filter_id, filter_ptr));
            samplereturn_msgs::NamedPoint point_msg;
            point_msg.header.frame_id = _filter_frame_id;
            point_msg.header.stamp = ros::Time::now();
            point_msg.point.x = filter_ptr->statePost.at<float>(0);
            point_msg.point.y = filter_ptr->statePost.at<float>(1);
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
    try
    {
        listener_.lookupTransform(_filter_frame_id, "base_link", ros::Time(0), transform);
    }
    catch(tf::TransformException e)
    {
        ROS_INFO_STREAM("Could not transform to " << _filter_frame_id << " : " << e.what());
        return;
    }
    float nearest_dist = 10000;
    float dist;
    int nearest_id = 0;
    std::shared_ptr<ColoredKF> nearest_filter;
    /* Walk list to find nearest good filter */
    if (filter_list_.size() > 0) {
      if (current_published_id_ == 0) {
        for (auto filter_ptr : filter_list_) {
          if ((filter_ptr->certainty > config_.pub_certainty_thresh) &&
              (filter_ptr->errorCovPost.at<float>(0,0) < config_.max_pub_cov)) {
            dist = sqrt(pow((transform.getOrigin().x()-filter_ptr->statePost.at<float>(0)),2) +
                   pow((transform.getOrigin().y()-filter_ptr->statePost.at<float>(1)),2));
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
      point_msg.point.x = nearest_filter->statePost.at<float>(0);
      point_msg.point.y = nearest_filter->statePost.at<float>(1);
      point_msg.point.z = 0;
      point_msg.filter_id = nearest_filter->filter_id;
      pub_detection.publish(point_msg);
    }
  }

  int addFilter(const samplereturn_msgs::NamedPoint& msg)
  {
    ROS_DEBUG("Adding New Filter");

    samplereturn::HueHistogram hh(msg.model.hue);
    std::shared_ptr<ColoredKF> CKF(new ColoredKF(config_, msg, filter_id_count_++));
    CKF->predict();
    CKF->measure(msg, config_.PDgO, config_.PDgo);
    filter_list_.push_back(CKF);
    ROS_DEBUG("Filter ID Count: %d", filter_id_count_);
    ROS_DEBUG("Initial Certainty: %f", CKF->certainty);
    return CKF->filter_id;
  }

  int checkObservation(const samplereturn_msgs::NamedPoint& msg)
  {
    ROS_DEBUG("Check Observation");

    for (size_t i=0; i<exclusion_list_.size(); i++) {
      float dist = sqrt(pow((std::get<0>(exclusion_list_[i]) - msg.point.x),2) +
        pow((std::get<1>(exclusion_list_[i]) - msg.point.y),2));
      if (dist < std::get<2>(exclusion_list_[i])) {
        return 0;
      }
    }

    cv::Mat meas_state(3, 1, CV_32F);
    meas_state.at<float>(0) = msg.point.x;
    meas_state.at<float>(1) = msg.point.y;
    meas_state.at<float>(2) = msg.point.z;

    for (auto ckf : filter_list_) {
      double dist = cv::norm(ckf->measurementMatrix * ckf->statePost
            - meas_state);
      samplereturn::HueHistogram hh(msg.model.hue);
      double distance = hh.distance(ckf->huemodel);
      bool color_check = (distance<config_.max_colormodel_distance);
      if ((dist < config_.max_dist) and color_check){
        ROS_DEBUG("Color Check Passed");
        ROS_DEBUG("Adding measurement to filter: %i", ckf->filter_id);
        ROS_DEBUG("Pre Meas Prob: %f",ckf->certainty);
        ckf->predict();
        ckf->measure(msg, config_.PDgO, config_.PDgo);
        ROS_DEBUG("Measurement Updated Prob: %f",ckf->certainty);
        return ckf->filter_id;
      }
      else if ((dist < config_.max_dist) and not color_check) {
        ROS_DEBUG("Color Check Failed");
        return 0;
      }
    }

    // close to no existing filter, add a new one
    return addFilter(msg);
  }

  /* This will check if each hypothesis is in view currently */
  bool isInView (const std::shared_ptr<ColoredKF>& kf) {
    ROS_DEBUG("Is In View Check");
    /* This is in base_link, transform it to odom */
    for(auto frustum : search_frusta_)
    {
      cv::Mat frustum_odom(frustum.second.rows,2,CV_32FC1);
      geometry_msgs::PointStamped temp_msg, temp_msg_odom;
      temp_msg.header.frame_id = "base_link";
      temp_msg.header.stamp = ros::Time(0);
      for (int i=0; i<frustum.second.rows; i++) {
        temp_msg.point.x = frustum.second.at<float>(i,0);
        temp_msg.point.y = frustum.second.at<float>(i,1);
        ROS_DEBUG("Frustum Points: X %f, Y: %f", temp_msg.point.x, temp_msg.point.y);
        temp_msg.point.z = 0.0;
        try {
          listener_.transformPoint(_filter_frame_id,temp_msg,temp_msg_odom);
        }
        catch (tf::TransformException e) {
          ROS_ERROR_STREAM("Aww shit " << e.what());
          return false;
        }
        frustum_odom.at<float>(i,0) = temp_msg_odom.point.x;
        frustum_odom.at<float>(i,1) = temp_msg_odom.point.y;
        ROS_DEBUG("Frustum_odom Points: X %f, Y: %f", temp_msg_odom.point.x, temp_msg_odom.point.y);
      }
      double retval = cv::pointPolygonTest(frustum_odom,
          cv::Point2f(kf->statePost.at<float>(0),kf->statePost.at<float>(1)), false);
      ROS_DEBUG("In View?: %f", retval);
      if (retval)
        return true;
      else
        continue;
    }
    return false;
  }

  void checkFilterAges() {
    filter_list_.erase(std::remove_if(filter_list_.begin(), filter_list_.end(),
        std::bind1st(std::mem_fun(&KalmanDetectionFilter::isOld),this)),
        filter_list_.end());
  }

  bool isOld (std::shared_ptr<ColoredKF> ckf) {
    ROS_DEBUG("Is Old Call");
    cv::Mat eigenvalues;
    cv::eigen(ckf->errorCovPost, eigenvalues);
    for (int i=0; i<eigenvalues.rows; i++) {
      if (eigenvalues.at<float>(i) > config_.max_cov) {
        ROS_DEBUG("Position Cov Too High. Removing Filter: %d",ckf->filter_id);
        clearMarker(ckf);
        return true;
      }
    }
    if (ckf->certainty < config_.min_certainty) {
      ROS_DEBUG("Certainty Too Low. Removing Filter: %d",ckf->filter_id);
      clearMarker(ckf);
      return true;
    }
    return false;
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
      cv::Point mean(filter_ptr->statePost.at<float>(0) * px_per_meter,
          filter_ptr->statePost.at<float>(1) * px_per_meter);
      float rad_x = filter_ptr->errorCovPost.at<float>(0,0) * px_per_meter;
      float rad_y = filter_ptr->errorCovPost.at<float>(1,1) * px_per_meter;
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
      ss << "H: " << filter_ptr->huemodel.dominant_hue() << "Prob: " << filter_ptr->certainty;
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
      cov.pose.position.x = filter_ptr->statePost.at<float>(0);
      cov.pose.position.y = filter_ptr->statePost.at<float>(1);
      cov.pose.position.z = 0.5;
      cov_text.pose.position.x = filter_ptr->statePost.at<float>(0);
      cov_text.pose.position.y = filter_ptr->statePost.at<float>(1);
      cov_text.pose.position.z = 1.0;
      cov.pose.orientation.x = 0;
      cov.pose.orientation.y = 0;
      cov.pose.orientation.z = 0;
      cov.pose.orientation.w = 1;
      cov.scale.x = filter_ptr->errorCovPost.at<float>(0,0);
      cov.scale.y = filter_ptr->errorCovPost.at<float>(1,1);
      cov.scale.z = 1.0;
      cov_text.scale.z = 0.5;
      cov.lifetime = ros::Duration();
      cov_text.lifetime = ros::Duration();
      marker_array.markers.push_back(cov);
      marker_array.markers.push_back(cov_text);
    }

    for (size_t i=0; i<exclusion_list_.size(); i++) {
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
      std::cout << "State: " << filter_ptr->statePost << std::endl;
    }
  }

};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_detection_filter");
  detection_filter::KalmanDetectionFilter kdf;
  ros::spin();
}
