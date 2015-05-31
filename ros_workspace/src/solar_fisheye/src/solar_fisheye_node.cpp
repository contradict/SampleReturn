#include <ros/ros.h>
#include "solar_fisheye/sun_finder.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solar_fisheye");

    solar_fisheye::SunFinder sf;

    ros::spin();
}
