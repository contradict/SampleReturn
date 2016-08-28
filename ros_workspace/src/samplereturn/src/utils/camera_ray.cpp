#include <samplereturn/camera_ray.h>
#include <tf_conversions/tf_eigen.h>

namespace samplereturn
{

void getCameraRay(const image_geometry::PinholeCameraModel& model, const cv::Point2d& pt, cv::Point3d* ray)
{
    cv::Point2d rect_point;
    rect_point = model.rectifyPoint(pt);
    ROS_DEBUG("Rect Point: %f, %f",rect_point.x,rect_point.y);
    *ray = model.projectPixelTo3dRay(rect_point);
}

Eigen::Vector3d intersectRayPlane(const Eigen::Vector3d ray,
  const Eigen::Vector3d ray_origin, const Eigen::Vector4d plane)
{
    // Plane defined as ax + by + cz + d = 0
    // Ray: P = P0 + tV
    // Plane: P.N + d = 0, where P is intersection point
    // t = -(P0.N + d)/(V.N) , P = P0 + tV
    float t = -(ray_origin.dot(plane.head(3)) + plane[3]) / (ray.dot(plane.head(3)));
    Eigen::Vector3d P = ray_origin + t*ray;
    return P;
}

bool projectRayToGround(const tf::Transformer& listener,
      const tf::Stamped<tf::Point> camera_ray,
      Eigen::Vector4d ground_plane, std::string ground_frame,
      tf::Stamped<tf::Point>* world_point)
{
    tf::StampedTransform camera_transform;
    // This is a static link, so Time(0) should be fine
    if (!listener.canTransform(ground_frame, camera_ray.frame_id_, camera_ray.stamp_)) {
        ROS_INFO("Couldn't transform %s to %s\n",camera_ray.frame_id_.c_str(),
                ground_frame.c_str());
        return false;
    }
    listener.lookupTransform(ground_frame, camera_ray.frame_id_,ros::Time(0),camera_transform);
    tf::Stamped<tf::Point> ground_frame_ray;
    listener.transformVector(ground_frame, camera_ray, ground_frame_ray);
    Eigen::Vector3d ray, ray_origin;
    tf::vectorTFToEigen(ground_frame_ray, ray);
    tf::vectorTFToEigen(camera_transform.getOrigin(), ray_origin);
    Eigen::Vector3d ground_v = intersectRayPlane(ray, ray_origin, ground_plane); 
    tf::vectorEigenToTF(ground_v, *world_point);
    world_point->frame_id_ = ground_frame;
    world_point->stamp_ = camera_ray.stamp_;
    return true;
}

}
