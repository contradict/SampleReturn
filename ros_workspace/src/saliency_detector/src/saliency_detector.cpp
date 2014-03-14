#include "saliency_detector_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saliency_detection");
  ros::NodeHandle nh;

  SaliencyDetector *saliency_detector new SaliencyDetector();

  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig> dr_srv;
  dynamic_reconfigure::Server<saliency_detector::saliency_detector_paramsConfig>::CallbackType cb;

  //params
  string img_topic;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("img_topic", img_topic, string(""));

  ros::Subscriber sub_img =
    nh.subscribe(img_topic.c_str(), 3, &SaliencyDetector::imgCallback, saliency_detector);

  while (nh.ok()) {
    ros::spin();
  }
}
