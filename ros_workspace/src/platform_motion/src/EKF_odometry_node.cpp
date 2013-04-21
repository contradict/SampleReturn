#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/gaussian.h>
#include <iostream>

#include "odometry/EKF_odometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");

    platform_motion::EKFOdometryNode on;

    on.init();

    ros::spin();

    return 0;
}
