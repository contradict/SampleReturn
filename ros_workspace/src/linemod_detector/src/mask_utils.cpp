#include <linemod_detector/mask_utils.h>

#include <ros/console.h>

namespace linemod_detector {

static bool
maskToHull(const cv::Mat& mask_in, std::vector<cv::Point> *hull)
{
    cv::Mat localcopy;
    mask_in.copyTo(localcopy); // findContours is destructive
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(localcopy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    double maxArea = 0;
    int max_idx = 0;
    for (size_t i=0; i<contours.size(); i++) {
        double area = cv::contourArea(cv::Mat(contours[i]));
        if (area > maxArea) {
            maxArea = area;
            max_idx = i;
        }
    }
    if(contours.size()>0)
    {
        cv::convexHull(contours[max_idx], *hull);
        return true;
    }
    else
    {
        return false;
    }
}

bool computeGripAngle(const cv::Mat& mask, cv::RotatedRect* griprect)
{
    std::vector<cv::Point> hull;
    if(!maskToHull(mask, &hull))
    {
        ROS_ERROR("Failed to generate hull from mask, cannot compute grip angle.");
        return false;
    }
    *griprect = cv::minAreaRect(hull);
    ROS_DEBUG("Measured angle: %f width: %f height: %f",
            griprect->angle, griprect->size.width, griprect->size.height);
    return true;
}

bool computeBoundingBox(const cv::Mat& mask, cv::Rect* rect)
{
    std::vector<cv::Point> hull;
    if(!maskToHull(mask, &hull))
    {
        ROS_ERROR("Failed to generate hull from mask, cannot compute bounding rect.");
        return false;
    }
    *rect = cv::boundingRect(hull);
    ROS_DEBUG("Measured width: %d height: %d", rect->size().width, rect->size().height);
    return true;
}

}
