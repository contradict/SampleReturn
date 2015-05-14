#include "solar_fisheye/sun_finder.h"

#include <opencv2/opencv.hpp>

namespace solar_fisheye {

SunFinder::SunFinder()
{
    ros::NodeHandle nh("~");
    it_ = new image_transport::ImageTransport(nh);
    sub_ = it_->subscribeCamera("image", 1, &SunFinder::imageCallback, this);

    reconfigure_server_.setCallback(boost::bind(&SunFinder::configure, this, _1, _2));
}

void
SunFinder::configure(SolarFisheyeConfig &config, uint32_t level)
{
    (void)level;
    config_ = config;
}

void
SunFinder::imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
        const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    ROS_DEBUG("Got an image");
    cv::Mat imageMat = cv_bridge::toCvShare(image_msg, "mono8")->image;
    // do cool stuff with imageMat and info_msg
}

}
