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
#include <samplereturn_msgs/NamedPointArray.h>
#include <samplereturn_msgs/PursuitResult.h>
#include <samplereturn/colormodel.h>

#include <dynamic_reconfigure/server.h>
#include <detection_filter/ManipulatorKalmanFilterConfig.h>
#include <detection_filter/probability.h>
#include <detection_filter/coloredkf.hpp>

/* This is going to subscribe to a detection channel, maintain some number
 * of Kalman filters for hypothesis, and publish confirmed detections
 * when the covariance and velocity of a filter fall below a threshold.
 * It will also age out filters that have too large a convariance.
 */

namespace detection_filter
{

class KalmanDetectionFilter
{
    ros::NodeHandle nh;
    ros::Subscriber sub_detections;
    ros::Publisher pub_detection;
    ros::Publisher pub_debug_img;

    ros::Subscriber sub_ack;

    detection_filter::ManipulatorKalmanFilterConfig config_;

    std::shared_ptr<ColoredKF> current_filter_;

    ros::Time last_negative_measurement_;
    bool updated_;

    int16_t filter_id_count_;

    tf::TransformListener listener_;

    std::string _filter_frame_id;

    dynamic_reconfigure::Server<detection_filter::ManipulatorKalmanFilterConfig> dr_srv;

    XmlRpc::XmlRpcValue color_transitions_;
    std::map<std::string,std::vector<std::string> > color_transitions_map_;

    /* Dynamic reconfigure callback */
    void configCallback(detection_filter::ManipulatorKalmanFilterConfig &config, uint32_t level);

    /* For incoming detections: assign to filter or create new filter
     * For each filter, predict, (update), check for deletion
     * Publish closest point to inspect
     * When done, delete and add exclusion zone, deleting other filters and inspection points
     */
    void detectionCallback(const samplereturn_msgs::NamedPointArrayConstPtr& msg);

    void publishFilteredDetections(void);

    void addFilter(const samplereturn_msgs::NamedPoint& msg);

    bool checkColor(const samplereturn::HueHistogram& filter, const samplereturn::HueHistogram& measurement);

    bool transformPointToFilter(samplereturn_msgs::NamedPoint& msg);

    bool checkObservation(const samplereturn_msgs::NamedPoint& msg);

    bool isOld(const std::shared_ptr<ColoredKF>& ckf);

    void checkFilterAges();

    void drawFilterStates();

    void printFilterState();

    public:
        KalmanDetectionFilter();
};

KalmanDetectionFilter::KalmanDetectionFilter()
{
    dynamic_reconfigure::Server<detection_filter::ManipulatorKalmanFilterConfig>::CallbackType cb;

    cb = boost::bind(&KalmanDetectionFilter::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("filter_frame_id", _filter_frame_id, std::string("odom"));

    last_negative_measurement_ = ros::Time(0);
    updated_ = false;

    filter_id_count_ = 0;
    sub_detections =
        nh.subscribe("point", 3, &KalmanDetectionFilter::detectionCallback, this);

    pub_detection =
        nh.advertise<samplereturn_msgs::NamedPoint>("filtered_point", 3);

    pub_debug_img =
        nh.advertise<sensor_msgs::Image>("debug_img", 3);

}

void
KalmanDetectionFilter::configCallback(detection_filter::ManipulatorKalmanFilterConfig &config, uint32_t level)
{
    (void)level;
    ROS_INFO("configCallback");

    config_ = config;

    if(config.clear_filters) {
        //clear all filters
        if(current_filter_)
            current_filter_.reset();
        config.clear_filters = false;
    }

}

void
KalmanDetectionFilter::detectionCallback(const samplereturn_msgs::NamedPointArrayConstPtr& msg)
{
    if(msg->header.stamp > last_negative_measurement_)
    {
        double dt = (msg->header.stamp - last_negative_measurement_).toSec();
        if(dt>config_.max_measurement_interval)
        {
            ROS_DEBUG("Large measurement interval, resetting");
            current_filter_.reset();
        }
        if(current_filter_ && !updated_)
        {
            ROS_DEBUG("Negative measurement");
            current_filter_->predict();
            current_filter_->errorCovPre.copyTo(current_filter_->errorCovPost);;
            current_filter_->certainty = updateProb(
                    current_filter_->certainty,
                    false,
                    config_.PDgO, config_.PDgo);
        }
        last_negative_measurement_ = msg->header.stamp;
        updated_ = false;
    }

    for(const auto & np : msg->points)
    {
        samplereturn_msgs::NamedPoint fp(np);
        if(!transformPointToFilter(fp))
            continue;

        if(!current_filter_)
        {
            addFilter(np);
            updated_ = true;
        }
        else
        {
            updated_ |= checkObservation(np);
        }
    }

    checkFilterAges();
    publishFilteredDetections();
    drawFilterStates();
    printFilterState();
}

void
KalmanDetectionFilter::publishFilteredDetections()
{
    if(!current_filter_)
        return;
    if(isOld(current_filter_))
        return;
    if (current_filter_->statePost.at<float>(3) < config_.max_pub_vel &&
            current_filter_->statePost.at<float>(4) < config_.max_pub_vel) {
        samplereturn_msgs::NamedPoint point_msg;
        current_filter_->toMsg(point_msg, ros::Time::now(), _filter_frame_id);
        pub_detection.publish(point_msg);
    }
}

void
KalmanDetectionFilter::addFilter(const samplereturn_msgs::NamedPoint& msg)
{
    current_filter_.reset(new ColoredKF(config_, msg, filter_id_count_++));
}

bool
KalmanDetectionFilter::checkColor(const samplereturn::HueHistogram& filter, const samplereturn::HueHistogram& measurement)
{
    if(config_.perform_color_check)
    {
        return filter.distance(measurement)<config_.max_colormodel_distance;
    }
    else
    {
        return true;
    }
}

bool
KalmanDetectionFilter::transformPointToFilter(samplereturn_msgs::NamedPoint& msg)
{
    tf::Point pt;
    tf::pointMsgToTF(msg.point, pt);
    tf::Stamped<tf::Point> msg_point(pt, msg.header.stamp, msg.header.frame_id);
    tf::Stamped<tf::Point> filter_point;
    if(!listener_.canTransform(_filter_frame_id, msg.header.frame_id, msg.header.stamp))
        return false;
    listener_.transformPoint(_filter_frame_id, msg_point, filter_point);
    tf::pointTFToMsg(filter_point, msg.point);
    msg.header.frame_id = _filter_frame_id;
    return true;
}

bool
KalmanDetectionFilter::checkObservation(const samplereturn_msgs::NamedPoint& msg)
{
    double dist = current_filter_->distance(msg);
    bool color_check = checkColor(current_filter_->huemodel, msg.model.hue);
    bool dist_check = dist < config_.max_dist;
    if (dist_check && color_check) {
        ROS_DEBUG("Adding measurement to filter");
        current_filter_->predict();
        current_filter_->measure(msg, config_.PDgO, config_.PDgo);
        current_filter_->huemodel = samplereturn::HueHistogram(msg.model.hue);
        return true;
    }
    else if (dist_check && !color_check)
    {
        ROS_DEBUG("Color Check Failed");
    }
    else
    {
        ROS_DEBUG("Distance check failed");
    }
    return false;
}

bool
KalmanDetectionFilter::isOld(const std::shared_ptr<ColoredKF>& ckf)
{
    cv::Mat eigenvalues;
    cv::eigen(ckf->errorCovPost, eigenvalues);
    for (int i=0; i<eigenvalues.rows; i++) {
        if (eigenvalues.at<float>(i) > config_.max_cov) {
            return true;
        }
    }
    return false;
}

void
KalmanDetectionFilter::checkFilterAges()
{
    if(current_filter_ && isOld(current_filter_))
    {
        current_filter_.reset();
        ROS_DEBUG("Filter aged out");
    }
}

void
KalmanDetectionFilter::drawFilterStates()
{
    if(!current_filter_)
        return;
    geometry_msgs::PointStamped temp_msg, temp_msg_base_link;
    temp_msg.header.frame_id = "odom";
    temp_msg.header.stamp = ros::Time(0);

    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    float px_per_meter = 50.0;
    float offset = 250;
    temp_msg.point.x = current_filter_->statePost.at<float>(0);
    temp_msg.point.y = current_filter_->statePost.at<float>(1);
    temp_msg.point.z = 0.0;

    if( listener_.canTransform( "base_link", temp_msg.header.frame_id, temp_msg.header.stamp))
    {
        listener_.transformPoint("base_link",temp_msg,temp_msg_base_link);
    }
    else
    {
        ROS_ERROR_STREAM("cannot transform filter from odom to base_link");
        return;
    }

    cv::Point mean(temp_msg_base_link.point.x * px_per_meter,
            temp_msg_base_link.point.y * px_per_meter);
    float rad_x = current_filter_->errorCovPost.at<float>(0,0) * px_per_meter;
    float rad_y = current_filter_->errorCovPost.at<float>(1,1) * px_per_meter;
    cv::circle(img, mean+cv::Point(0,offset), 5, cv::Scalar(255,0,0));
    cv::ellipse(img, mean+cv::Point(0,offset), cv::Size(rad_x, rad_y), 0, 0, 360, cv::Scalar(0,255,0));

    //printFilterState();
    std_msgs::Header header;
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",img).toImageMsg();
    pub_debug_img.publish(debug_img_msg);
}

void
KalmanDetectionFilter::printFilterState()
{
    if(!current_filter_)
        return;

    std::stringstream ss;
    ss << "id " << current_filter_->filter_id
       << " state " << current_filter_->statePost.at<float>(0)
                    << ", "  << current_filter_->statePost.at<float>(1)
                    << ", "  << current_filter_->statePost.at<float>(2)
                    << ", "  << sqrt(pow(current_filter_->statePost.at<float>(3), 2) +
                                 pow(current_filter_->statePost.at<float>(4), 2) +
                                 pow(current_filter_->statePost.at<float>(5), 2))
       << " cov " << current_filter_->errorCovPost.at<float>(0,0)
                  << ", " << current_filter_->errorCovPost.at<float>(1,1);
    ROS_DEBUG_STREAM(ss.str());
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_detection_filter");
    detection_filter::KalmanDetectionFilter kdf;
    ros::spin();
}
