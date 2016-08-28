#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <basler_camera/CameraConfig.h>
#include <dynamic_reconfigure/server.h>
#include <platform_motion_msgs/Enable.h>
#include <std_msgs/Bool.h>
#include <ros/timer.h>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>


namespace BaslerCamera
{

class BaslerNode : public Pylon::CBaslerUsbImageEventHandler
{
    std::string camera_info_url;
    std::string frame_id;
    std::string serial_number;
    std::string camera_name;

    bool grab_enabled;
    bool publish_enabled;
    ros::ServiceServer grab_service;
    ros::Subscriber grab_topic;
    ros::ServiceServer enable_service;
    ros::Subscriber enable_topic;

    image_transport::ImageTransport *it_;
    image_transport::CameraPublisher cam_pub_;
    camera_info_manager::CameraInfoManager *cinfo_manager_;
    dynamic_reconfigure::Server<basler_camera::CameraConfig> server;

    basler_camera::CameraConfig config_;

    Pylon::CImageFormatConverter converter_;
    Pylon::CPylonImage pylon_image_;
    std::string output_encoding;
    int output_bytes_per_pixel;
    double initial_stamp_;
    uint64_t initial_counter_;

    // Automatically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    Pylon::CBaslerUsbInstantCamera camera;

    void
    find_camera(void);

    bool
    handle_basler_parameter(std::string name, bool value);
    int
    handle_basler_parameter(std::string name, int value);
    double
    handle_basler_parameter(std::string name, double value);
    std::string
    handle_basler_parameter(std::string name, std::string value);

    void
    configure_callback(basler_camera::CameraConfig &config, uint32_t level);

    bool
    service_enable_grab(platform_motion_msgs::Enable::Request &req, platform_motion_msgs::Enable::Response &resp);
    void
    topic_enable_grab(std_msgs::BoolConstPtr msg);
    bool
    do_enable_grab(bool state);
    bool
    service_enable_publish(platform_motion_msgs::Enable::Request &req, platform_motion_msgs::Enable::Response &resp);
    void
    topic_enable_publish(std_msgs::BoolConstPtr msg);

    void set_timestamp_offset(void);

    public:
    BaslerNode(ros::NodeHandle &nh);

    void shutdown(void);

    virtual void OnImageGrabbed( Pylon::CBaslerUsbInstantCamera& camera, const Pylon::CBaslerUsbGrabResultPtr& ptrGrabResult);
    virtual void OnImagesSkipped( Pylon::CBaslerUsbInstantCamera& camera, size_t countOfSkippedImages)
    {
        (void)camera;
        ROS_ERROR_STREAM (countOfSkippedImages  << " images have been skipped.");
    }
};

}
