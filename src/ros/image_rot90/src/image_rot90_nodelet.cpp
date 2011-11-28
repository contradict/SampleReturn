//vim: set sw=4 ts=4 et:
#include <ros/ros.h>
#include <ros/time.h>

#include <dynamic_reconfigure/server.h>

#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core/core.hpp>

#include "image_rot90/image_rot90.h"

namespace image_rot90{
    
void ImageRot90::onInit() {
  ros::NodeHandle node = getNodeHandle();
  ros::NodeHandle pnode = getPrivateNodeHandle();

  pnode.getParam("rot", rot);

  it = new image_transport::ImageTransport(node);
  pub = it->advertise("rot90/image_raw_rot", 1);
  sub = it->subscribe("rot90/image_raw_in", 1, &ImageRot90::imageCallback, this);

  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  srv.reset(
          new dynamic_reconfigure::Server<DynamicParametersConfig>(
              dynamic_reconfigure_mutex,
              private_nh));
  dynamic_reconfigure::Server<DynamicParametersConfig>::CallbackType f;
  f = boost::bind(&ImageRot90::configureCallback, this, _1, _2);
  srv->setCallback(f);

}

void ImageRot90::configureCallback(DynamicParametersConfig &config, uint32_t level)
{
    rot = config.rot;
    ROS_INFO("Set rot to %s", rot.c_str());
}

void ImageRot90::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    std::string encoding;
    try
    {
        if (sensor_msgs::image_encodings::isColor(msg->encoding)) {
            encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr = cv_bridge::toCvShare(msg, encoding);
        } else {
            encoding = sensor_msgs::image_encodings::MONO8;
            cv_ptr = cv_bridge::toCvShare(msg, encoding);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImage cv_img_rotated;
    cv_img_rotated.header = msg->header;
    cv_img_rotated.encoding = encoding;
    if( 0==rot.compare(0, 5, "-pi/2") || 0==rot.compare(0, 3, "-90") ) {
        // Transpose, the flip about X to rotate -90
        cv::flip(cv_ptr->image.t(), cv_img_rotated.image, 0);
        // Transpose, the flip about Y to rotate 90
    }else if( 0==rot.compare(0, 4, "pi/2") || 0==rot.compare(0, 2, "90") ) {
        cv::flip(cv_ptr->image.t(), cv_img_rotated.image, 1);
    }else if( 0==rot.compare(0, 2, "pi") || 0==rot.compare(0, 3, "180") ) {
        // flip about X and Y to rotate 180
        cv::flip(cv_ptr->image, cv_img_rotated.image, -1);
    }else if( 0==rot.compare(0, 5, "flipx") ) {
        // flip about X only
        cv::flip(cv_ptr->image, cv_img_rotated.image, 0);
    }else if( 0==rot.compare(0, 5, "flipy") ) {
        // flip about Y only
        cv::flip(cv_ptr->image, cv_img_rotated.image, 1);
    }else if( 0==rot.compare(0, 9, "transpose") ) {
        // transpose only
        cv_img_rotated.image = cv_ptr->image.t();
    }else {
        //unrecognized do nothing
        cv_img_rotated.image = cv_ptr->image;
    }

    pub.publish(cv_img_rotated.toImageMsg());
}

ImageRot90::~ImageRot90() {
    if(it) delete it;
}

};
