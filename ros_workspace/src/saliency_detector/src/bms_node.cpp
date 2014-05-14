#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "BMS.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_listener");
  BMS_Node bn;
  ros::spin();
}
