#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>

namespace samplereturn
{
void getCameraRay(const image_geometry::PinholeCameraModel& model, const cv::Point2d& pt, cv::Point3d* ray);

Eigen::Vector3d intersectRayPlane(const Eigen::Vector3d ray,
  const Eigen::Vector3d ray_origin, const Eigen::Vector4d plane);

bool projectRayToGround(const tf::Transformer& listener,
      const tf::Stamped<tf::Point> camera_ray,
      Eigen::Vector4d ground_plane, std::string ground_frame,
      tf::Stamped<tf::Point>* world_point);
}

