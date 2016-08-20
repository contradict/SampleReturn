#include <Eigen/Dense>
#include <ros/ros.h>
#include <samplereturn_msgs/PatchArray.h>
#include <platform_motion_msgs/Enable.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <linemod_detector/mask_utils.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <linemod_detector/PointCloudProjectorConfig.h>

namespace linemod_detector {

class PointCloudProjector {
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Subscriber<samplereturn_msgs::PatchArray> patch_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, samplereturn_msgs::PatchArray> ExactSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> sync;
    ros::ServiceServer enable_service;
    dynamic_reconfigure::Server<PointCloudProjectorConfig> reconfigure_server;

    bool enabled_;

    PointCloudProjectorConfig config_;

    tf::TransformListener listener_;

    ros::Publisher patches_out;
    ros::Publisher debug_points_out;

    std::string clipping_frame_id_;

    void
    synchronized_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg,
            const samplereturn_msgs::PatchArrayConstPtr& patches_msg);
    bool
    enable(platform_motion_msgs::Enable::Request& req, platform_motion_msgs::Enable::Response& resp);
    void
    configure_callback(PointCloudProjectorConfig &config, uint32_t level);

    public:
    PointCloudProjector(ros::NodeHandle nh);
};

PointCloudProjector::PointCloudProjector(ros::NodeHandle nh) :
    pointcloud_sub(nh, "pointcloud", 1),
    patch_sub(nh, "patches", 1),
    sync(ExactSyncPolicy(50), pointcloud_sub, patch_sub)
{
    patches_out = nh.advertise<samplereturn_msgs::PatchArray>("positioned_patches", 1);
    debug_points_out = nh.advertise<sensor_msgs::PointCloud2>("debug_pointcloud", 1);
    sync.registerCallback(boost::bind(&PointCloudProjector::synchronized_callback, this, _1, _2));
    ros::NodeHandle pnh("~");
    pnh.param("clipping_frame_id", clipping_frame_id_, std::string("base_link"));
    if(!pnh.param("start_enabled", true))
    {
        enabled_ = true;
        pointcloud_sub.subscribe(nh, "pointcloud", 1);
        patch_sub.subscribe(nh, "patches", 1);
    }
    else
    {
        enabled_ = false;
    }
    reconfigure_server.setCallback(boost::bind(&PointCloudProjector::configure_callback, this,  _1, _2));
    enable_service = pnh.advertiseService( "enable", &PointCloudProjector::enable, this);
}

void
PointCloudProjector::configure_callback(PointCloudProjectorConfig &config, uint32_t level)
{
    config_ = config;
}

bool
PointCloudProjector::enable(platform_motion_msgs::Enable::Request& req, platform_motion_msgs::Enable::Response& resp)
{
    ros::NodeHandle nh;
    if(req.state && !enabled_)
    {
        pointcloud_sub.subscribe(nh, "pointclouds", 1);
        patch_sub.subscribe(nh, "patches", 1);
    }
    else if(enabled_)
    {
        pointcloud_sub.unsubscribe();
        patch_sub.unsubscribe();
    }
    enabled_ = req.state;
    resp.state = enabled_;
    return true;
}

void
PointCloudProjector::synchronized_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg,
        const samplereturn_msgs::PatchArrayConstPtr& patches_msg)
{
    samplereturn_msgs::PatchArray positioned_patches;
    positioned_patches.header = patches_msg->header;
    positioned_patches.cam_info = patches_msg->cam_info;

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(patches_msg->cam_info);

    if(!listener_.canTransform(clipping_frame_id_, patches_msg->header.frame_id,
                patches_msg->header.stamp))
        return;

    tf::StampedTransform camera;
    listener_.lookupTransform(clipping_frame_id_, patches_msg->header.frame_id,
            patches_msg->header.stamp, camera);
    Eigen::Vector3d camera_origin;
    tf::vectorTFToEigen(camera.getOrigin(), camera_origin);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorpoints(new pcl::PointCloud<pcl::PointXYZRGB>),
        points_native(new pcl::PointCloud<pcl::PointXYZRGB>),
        points_scaled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*points_msg, *points_native);
    Eigen::Transform<float, 3, Eigen::Affine> trans;
    trans.setIdentity();
    trans.scale(config_.pointcloud_scale);
    pcl::transformPointCloud(*points_native, *points_scaled, trans);
    pcl_ros::transformPointCloud(clipping_frame_id_, *points_scaled, *colorpoints, listener_);

    for(const auto& patch : patches_msg->patch_array)
    {
        samplereturn_msgs::Patch positioned_patch(patch);
        cv_bridge::CvImagePtr cv_ptr_mask;
        try {
            cv_ptr_mask = cv_bridge::toCvCopy(patch.mask, "mono8");
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge mask exception: %s", e.what());
            continue;
        }
        cv::Rect rect;
        computeBoundingBox(cv_ptr_mask->image, &rect);

        std::vector<cv::Point2d> rpoints;
        rpoints.push_back(cv::Point2d(rect.x,            rect.y));
        rpoints.push_back(cv::Point2d(rect.x,            rect.y+rect.height));
        rpoints.push_back(cv::Point2d(rect.x+rect.width, rect.y+rect.height));
        rpoints.push_back(cv::Point2d(rect.x+rect.width, rect.y));
        std::vector<Eigen::Vector3d> rays;
        std::transform(rpoints.begin(), rpoints.end(), rays.begin(),
                [model, patches_msg, this](cv::Point2d uv) -> Eigen::Vector3d
                {
                    cv::Point3d xyz = model.projectPixelTo3dRay(uv);
                    tf::Stamped<tf::Vector3> vect(tf::Vector3(xyz.x, xyz.y, xyz.z),
                        patches_msg->header.stamp,
                        patches_msg->header.frame_id);
                    tf::Stamped<tf::Vector3> vect_t;
                    listener_.transformVector(clipping_frame_id_, vect, vect_t);
                    Eigen::Vector3d ray;
                    tf::vectorTFToEigen(vect_t, ray);
                    return ray;
                });

        pcl::PointIndices::Ptr clipped(new pcl::PointIndices);
        for(int i=0;i<4;i++)
        {
            Eigen::Vector4f plane;
            plane.segment<3>(0) = rays[i].cross(rays[(i+1)%4]).cast<float>();
            plane[3] = -float(camera_origin[2]);
            pcl::PlaneClipper3D<pcl::PointXYZ> clip(plane);
            clip.clipPointCloud3D(points,  clipped->indices, clipped->indices);
        }

        if(debug_points_out.getNumSubscribers()>0)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorpoints(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*points_msg, *colorpoints);
            pcl::PointCloud<pcl::PointXYZRGB> clipped_pts;
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(colorpoints);
            extract.setIndices(clipped);
            extract.filter(clipped_pts);
            sensor_msgs::PointCloud2 clipped_msg;
            pcl::toROSMsg(clipped_pts, clipped_msg);
            debug_points_out.publish( clipped_msg );
        }

        positioned_patches.patch_array.push_back(positioned_patch);
    }

    patches_out.publish( positioned_patches );
}

/*

void publishPoint(cv::RotatedRect rect, std::string class_id,
      std_msgs::Header header, int sample_id)
{
    ROS_DEBUG("Median Disp: %f", median_disp);

    cam_model_.projectDisparityTo3d(cv::Point2d(rect.center.x,rect.center.y),
            median_disp, xyz);
    temp_point.point.x = xyz.x;
    temp_point.point.y = xyz.y;
    ROS_DEBUG("Projected z: %f", xyz.z);
    if (xyz.z > max_depth) {
        temp_point.point.z = max_depth;
        temp_point.point.x /= (xyz.z/max_depth);
        temp_point.point.y /= (xyz.z/max_depth);
    }
    else if (xyz.z < min_depth) {
        temp_point.point.z = min_depth;
        temp_point.point.x /= (xyz.z/min_depth);
        temp_point.point.y /= (xyz.z/min_depth);
    }
    else {
        temp_point.point.z = xyz.z;
    }
    bool wait =
        listener_.waitForTransform(_detection_frame_id, header.frame_id, header.stamp, ros::Duration(0.03));
    if (!wait) {
        return;
    }
    listener_.transformPoint(_detection_frame_id, temp_point, odom_point);

    point_msg.name = class_id;
    point_msg.sample_id = sample_id;
    point_msg.header = header;
    point_msg.header.frame_id = _detection_frame_id;
    point_msg.grip_angle = rect.angle;
    point_msg.point = odom_point.point;
    LineMOD_Detector::point_pub.publish(point_msg);
}
*/

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointCloudProjector");
    ros::NodeHandle nh;
    linemod_detector::PointCloudProjector proj(nh);
    ros::spin();
}

