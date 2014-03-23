#include "saliency_detector_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detection");
  ros::NodeHandle nh;

  SaliencyDetector *saliency_detector = new SaliencyDetector();

  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;
  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

  cb = boost::bind(&SaliencyDetector::configCallback, saliency_detector, _1, _2);
  dr_srv.setCallback(cb);

  //params
  string img_topic;
  string bms_debug_topic;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("img_topic", img_topic, string("/cam_img"));
  private_node_handle_.param("bms_debug_topic", bms_debug_topic, string("/bms_img"));

  ros::Subscriber sub_img =
    nh.subscribe(img_topic.c_str(), 3, &SaliencyDetector::messageCallback, saliency_detector);

  ros::Publisher pub_bms_img =
    nh.advertise<sensor_msgs::Image>(bms_debug_topic.c_str(), 3);

  while (nh.ok()) {
    ros::spin();
  }
}
