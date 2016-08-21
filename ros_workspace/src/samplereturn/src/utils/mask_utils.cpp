#include <samplereturn/mask_utils.h>

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

double unwrap90(double angle)
{
    while(angle>M_PI/2)
        angle -= M_PI;
    while(angle<-M_PI/2)
        angle += M_PI;
    return angle;
}

bool computeGripAngle(const cv::Mat& mask, cv::RotatedRect* griprect, float *grip_angle)
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
    if(griprect->size.width>griprect->size.height)
    {
        *grip_angle = -griprect->angle*M_PI/180+M_PI_2;
    }
    else
    {
        *grip_angle = -griprect->angle*M_PI/180;
    }
    *grip_angle = unwrap90(*grip_angle);
    ROS_DEBUG("reported angle: %f", *grip_angle);

    return true;
}

void drawGripRect(cv::Mat& debug_image, const cv::RotatedRect& griprect)
{
    cv::Mat pts(4,1,CV_32FC2);
    griprect.points((cv::Point2f*)pts.data);
    pts.convertTo( pts, CV_32S);
    cv::polylines(debug_image, (cv::Point2i**)&pts.data, &pts.rows, 1, true, cv::Scalar(255,0,0), 3, CV_AA, 0);
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
