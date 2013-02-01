//vim: set sw=4 ts=4 et:
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_transport/image_transport.h"

#include "sensor_msgs/Image.h"

#include "image_rot90/DynamicParametersConfig.h"

namespace image_rot90 {

class ImageRot90 : public nodelet::Nodelet {
    public:
        ImageRot90() {};
        void onInit();
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void configureCallback(DynamicParametersConfig &config, uint32_t level);
        ~ImageRot90();

    private:
        image_transport::ImageTransport *it;
        image_transport::Publisher pub;
        image_transport::Subscriber sub;

        boost::recursive_mutex dynamic_reconfigure_mutex;
        boost::shared_ptr<dynamic_reconfigure::Server<DynamicParametersConfig> > srv;

        std::string rot;
};
};

PLUGINLIB_DECLARE_CLASS(image_rot90, ImageRot90, image_rot90::ImageRot90, nodelet::Nodelet);
