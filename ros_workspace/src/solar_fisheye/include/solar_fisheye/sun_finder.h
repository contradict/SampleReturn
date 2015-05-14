#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include <solar_fisheye/SolarFisheyeConfig.h>

namespace solar_fisheye {
class SunFinder
{
    image_transport::ImageTransport *it_;
    image_transport::CameraSubscriber sub_;

    SolarFisheyeConfig config_;
    dynamic_reconfigure::Server<solar_fisheye::SolarFisheyeConfig> reconfigure_server_;

    void configure(SolarFisheyeConfig &config, uint32_t level);

    void imageCallback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);

    public:
        SunFinder();
};
}
