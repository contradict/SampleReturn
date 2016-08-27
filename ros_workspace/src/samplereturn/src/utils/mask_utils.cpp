#include <samplereturn/mask_utils.h>
#include <samplereturn/camera_ray.h>

#include <ros/console.h>
#include <tf/tf.h>

namespace samplereturn
{

static bool
findLargestContour(const cv::Mat& mask_in, std::vector<cv::Point>* largest_contour)
{
    cv::Mat localcopy;
    mask_in.copyTo(localcopy); // findContours is destructive
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(localcopy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    auto max_it = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) -> bool
            {
                double a_area = cv::contourArea(cv::Mat(a));
                double b_area = cv::contourArea(cv::Mat(b));
                return a_area < b_area;
            });
    int sz = max_it->size();
    largest_contour->resize(sz);
    std::copy(max_it->begin(), max_it->end(), largest_contour->begin());
    return sz>0;
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
    std::vector<cv::Point> largest_contour;
    if(!findLargestContour(mask, &largest_contour))
    {
        ROS_ERROR("Failed to generate any contours from mask, cannot compute grip angle.");
        return false;
    }
    *griprect = cv::minAreaRect(largest_contour);
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
    std::vector<cv::Point> largest_contour;
    if(!findLargestContour(mask, &largest_contour))
    {
        ROS_ERROR("Failed to find any contours in mask, cannot compute bounding rect.");
        return false;
    }
    *rect = cv::boundingRect(largest_contour);
    ROS_DEBUG("Measured width: %d height: %d", rect->size().width, rect->size().height);
    return true;
}

static void getMajorPoints(const cv::RotatedRect rect,
  cv::Point2d* major_point_a, cv::Point2d* major_point_b)
{
    cv::Point2f rect_points[4];
    rect.points(rect_points);
    float d0 = cv::norm(rect_points[0] - rect_points[1]);
    float d1 = cv::norm(rect_points[1] - rect_points[2]);
    if(d0>d1)
    {
        *major_point_a = (rect_points[0] + rect_points[3])/2.0f;
        *major_point_b = (rect_points[1] + rect_points[2])/2.0f;
    }
    else
    {
        *major_point_a = (rect_points[0] + rect_points[1])/2.0f;
        *major_point_b = (rect_points[2] + rect_points[3])/2.0f;
    }
}

bool computeMaskPositionAndSize(const tf::Transformer& listener,
      const cv::Mat& mask, const cv::Point2f roi_offset,
      const image_geometry::PinholeCameraModel& model, const ros::Time& stamp, const std::string& camera_frame_id,
      const Eigen::Vector4d& ground_plane, const std::string& ground_frame,
      float* dimension, tf::Stamped<tf::Point>* world_point,
      cv::Mat* debug_image)
{
    // compute oriented min area rectangle around mask
    cv::RotatedRect griprect;
    float grip_angle;
    if(!samplereturn::computeGripAngle(mask, &griprect, &grip_angle))
    {
        ROS_DEBUG("No contours found in patch");
        return false;
    }
    // place rectangle in full image
    griprect.center += roi_offset;
    // draw if needed
    if (debug_image) {
        samplereturn::drawGripRect(*debug_image, griprect);
    }

    // compute points at ends of major axis on midpoints of minor axis
    cv::Point2d major_point_a, major_point_b;
    getMajorPoints(griprect, &major_point_a, &major_point_b);
    ROS_DEBUG("Major Point A: %f, %f",major_point_a.x,major_point_a.y);
    ROS_DEBUG("Major Point B: %f, %f",major_point_b.x,major_point_b.y);

    // project major points to 3D rays in camera frame
    cv::Point3d ray_a, ray_b;
    getCameraRay(model, major_point_a, &ray_a);
    getCameraRay(model, major_point_b, &ray_b);
    ROS_DEBUG("Ray A %f, %f, %f",ray_a.x,ray_a.y,ray_a.z);
    ROS_DEBUG("Ray B %f, %f, %f",ray_b.x,ray_b.y,ray_b.z);

    // intersect rays with ground plane in "base_link"
    tf::Stamped<tf::Point> camera_ray_a(
            tf::Point(ray_a.x, ray_a.y, ray_a.z),
            stamp, camera_frame_id);
    tf::Stamped<tf::Point> camera_ray_b(
            tf::Point(ray_b.x, ray_b.y, ray_b.z),
            stamp, camera_frame_id);
    tf::Stamped<tf::Point> ground_point_a, ground_point_b;
    bool projected = (projectRayToGround(listener, camera_ray_a,
                ground_plane, ground_frame, &ground_point_a) &&
            projectRayToGround(listener, camera_ray_b,
                ground_plane, ground_frame, &ground_point_b));
    if(!projected)
    {
        return false;
    }
    ROS_DEBUG("Ground Point A: %f, %f, %f",ground_point_a.x(),ground_point_a.y(),ground_point_a.z());
    ROS_DEBUG("Ground Point B: %f, %f, %f",ground_point_b.x(),ground_point_b.y(),ground_point_b.z());

    // compute midpoint on ground, this is where the sample will be placed
    tf::Vector3 world_v = (ground_point_a + ground_point_b)/2;
    *world_point = tf::Stamped<tf::Point>(world_v, stamp, ground_frame);
    ROS_DEBUG("Ground Point Mid: %f, %f, %f", world_point->x(),world_point->y(),world_point->z());

    // compute distance between points on ground, this is the sample size
    *dimension = (ground_point_b - ground_point_a).length();
    ROS_DEBUG("Major axis size: %f", *dimension);
    return true;
}


}
