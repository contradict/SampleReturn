#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>

namespace samplereturn
{

bool computeGripAngle(const cv::Mat& mask, cv::RotatedRect* griprect, float *grip_angle);
bool computeBoundingBox(const cv::Mat& mask, cv::Rect* rect);
void drawGripRect(cv::Mat& debug_image, const cv::RotatedRect& griprect);
Eigen::Vector3d intersectRayPlane(const Eigen::Vector3d ray,
  const Eigen::Vector3d ray_origin, const Eigen::Vector4d plane);
bool computeMaskPositionAndSize(const tf::Transformer& listener,
      const cv::Mat& mask, const cv::Point2f roi_offset,
      const image_geometry::PinholeCameraModel& model, const ros::Time& stamp, const std::string& camera_frame_id,
      const Eigen::Vector4d& ground_plane, const std::string& ground_frame,
      float* dimension, float *angle, tf::Stamped<tf::Point>* world_point,
      cv::Mat* debug_image);

}
