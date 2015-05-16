#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <solar_fisheye/SolarFisheyeConfig.h>

namespace solar_fisheye {

void unprojectPointsFisheye( cv::InputArray distorted, cv::OutputArray undistorted, cv::InputArray K, cv::InputArray D, cv::InputArray R, cv::InputArray P);

class SunFinder
{
    image_transport::ImageTransport *it_;
    image_transport::CameraSubscriber sub_;
    image_geometry::PinholeCameraModel model_;


    SolarFisheyeConfig config_;
    dynamic_reconfigure::Server<solar_fisheye::SolarFisheyeConfig> reconfigure_server_;

    void configure(SolarFisheyeConfig &config, uint32_t level);

    void imageCallback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);

    public:
        SunFinder();
};
}
