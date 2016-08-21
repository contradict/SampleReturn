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
#include <samplereturn/mask_utils.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <linemod_detector/PointCloudProjectorConfig.h>
#include <visualization_msgs/MarkerArray.h>

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
    ros::Publisher debug_marker_out;

    std::string clipping_frame_id_;
    std::string output_frame_id_;

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
    debug_marker_out = nh.advertise<visualization_msgs::MarkerArray>("debug_marker", 1);
    sync.registerCallback(boost::bind(&PointCloudProjector::synchronized_callback, this, _1, _2));
    ros::NodeHandle pnh("~");
    pnh.param("clipping_frame_id", clipping_frame_id_, std::string("base_link"));
    pnh.param("output_frame_id", output_frame_id_, std::string("odom"));
    if(pnh.param("start_enabled", true))
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
    (void)level;
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
    // create patch output message
    samplereturn_msgs::PatchArray positioned_patches;
    positioned_patches.header = patches_msg->header;
    positioned_patches.cam_info = patches_msg->cam_info;

    // create marker array debug message
    visualization_msgs::MarkerArray vis_markers;

    // create camera model object
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(patches_msg->cam_info);

    // ensure tf is ready
    if(!listener_.canTransform(clipping_frame_id_, patches_msg->header.frame_id,
                patches_msg->header.stamp))
        return;

    // get camera origin in clipping frame
    tf::StampedTransform camera;
    listener_.lookupTransform(clipping_frame_id_, patches_msg->header.frame_id,
            patches_msg->header.stamp, camera);
    Eigen::Vector3d camera_origin;
    tf::vectorTFToEigen(camera.getOrigin(), camera_origin);

    // scale and transform pointcloud into clipping frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorpoints(new pcl::PointCloud<pcl::PointXYZRGB>),
        points_native(new pcl::PointCloud<pcl::PointXYZRGB>),
        points_scaled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*points_msg, *points_native);
    // this scale is a horible hack to fix the manipulator point clouds
    Eigen::Transform<float, 3, Eigen::Affine> trans;
    trans.setIdentity();
    trans.scale(config_.pointcloud_scale);
    pcl::transformPointCloud(*points_native, *points_scaled, trans);
    pcl_ros::transformPointCloud(clipping_frame_id_, *points_scaled, *colorpoints, listener_);

    // id counter for debug markers
    int mid = 0;
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

        // find bounding box of mask
        cv::Rect rect;
        computeBoundingBox(cv_ptr_mask->image, &rect);

        // turn image space bounding box into 4 3d rays
        cv::Point2d patch_origin(patch.image_roi.x_offset,
                patch.image_roi.y_offset);
        std::vector<cv::Point2d> rpoints;
        rpoints.push_back(cv::Point2d(rect.x,            rect.y) +
                patch_origin);
        rpoints.push_back(cv::Point2d(rect.x,            rect.y+rect.height) +
                patch_origin);
        rpoints.push_back(cv::Point2d(rect.x+rect.width, rect.y+rect.height) +
                patch_origin);
        rpoints.push_back(cv::Point2d(rect.x+rect.width, rect.y) +
                patch_origin);
        std::vector<Eigen::Vector3d> rays;
        rays.resize(rpoints.size());
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

        // clip point cloud by the planes of the bounding volume
        // described by the rays above
        // Add one more clipping plane at z=0 in the clipping frame
        // to remove noise points below the ground
        pcl::PointIndices::Ptr clipped(new pcl::PointIndices);
        for(size_t i=0;i<rays.size()+1;i++)
        {
            Eigen::Vector4f plane;
            if(i<rays.size())
            {
                plane.segment<3>(0) = -rays[i].cross(rays[(i+1)%4]).cast<float>();
                plane[3] = -plane.segment<3>(0).dot(camera_origin.cast<float>());
            }
            else
            {
                plane << 0,0,1, config_.bottom_clipping_depth;
            }
            pcl::PlaneClipper3D<pcl::PointXYZRGB> clip(plane);
            std::vector<int> newclipped;
            clip.clipPointCloud3D(*colorpoints,  newclipped, clipped->indices);
            clipped->indices.resize(newclipped.size());
            std::copy(newclipped.begin(), newclipped.end(),
                    clipped->indices.begin());
        }

        // publish clipped pointcloud if anybody is listening
        if(debug_points_out.getNumSubscribers()>0)
        {
            pcl::PointCloud<pcl::PointXYZRGB> clipped_pts;
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(colorpoints);
            extract.setIndices(clipped);
            extract.filter(clipped_pts);
            sensor_msgs::PointCloud2 clipped_msg;
            pcl::toROSMsg(clipped_pts, clipped_msg);
            debug_points_out.publish( clipped_msg );
        }

        // compute suitable z value for this patch
        // First, find min, max and sum
        typedef std::tuple<float, float, float> stats_tuple;
        stats_tuple z_stats = std::accumulate(
                clipped->indices.begin(), clipped->indices.end(),
                std::make_tuple(std::numeric_limits<float>().max(),
                                std::numeric_limits<float>().min(),
                                0.0f),
                [colorpoints](stats_tuple sum, int idx) -> stats_tuple
                {
                    return std::make_tuple(
                        std::min(std::get<0>(sum), colorpoints->points[idx].z),
                        std::max(std::get<1>(sum), colorpoints->points[idx].z),
                        std::get<2>(sum) + colorpoints->points[idx].z);
                });
        // use sum to find mean
        float z_min = std::get<0>(z_stats);
        float z_max = std::get<1>(z_stats);
        float z_mean = std::get<2>(z_stats)/float(clipped->indices.size());
        // build histogram of values larger than mean
        float hist_min = z_min + (z_mean-z_min)*config_.hist_min_scale;
        ROS_DEBUG("z_min: %04.3f z_mean: %04.3f z_max: %04.3f z_sum: %4.2f hist_min:%04.3f",
                z_min, z_mean, z_max, std::get<2>(z_stats), hist_min);
        const int NHIST = 20;
        int z_hist[NHIST];
        bzero(z_hist, NHIST*sizeof(int));
        std::accumulate(clipped->indices.begin(), clipped->indices.end(), z_hist,
                [colorpoints, hist_min, z_max](int *z_hist, int idx) -> int *
                {
                    float z = colorpoints->points[idx].z;
                    if(z>hist_min)
                    {
                        int zidx = floor((z-hist_min)*NHIST/(z_max-hist_min));
                        z_hist[zidx] ++;
                    }
                    return z_hist;
                });
        char debughist[2048];
        int pos=0;
        for(int i=0;i<NHIST;i++)
        {
            pos += snprintf(debughist+pos, 2048-pos, "%d, ", z_hist[i]);
        }
        debughist[pos] = '\x00';
        ROS_DEBUG("hist: %s", debughist);
        // select the most common value larger than the mean
        int * argmax = std::max_element( z_hist, z_hist + NHIST );
        ROS_DEBUG("argmax: %d", int(argmax - z_hist));
        float z_peak = (argmax - z_hist)*(z_max - hist_min)/NHIST + hist_min;
        if(z_peak>config_.maximum_patch_height)
        {
            ROS_INFO("Clipping z to max, was %f", z_peak);
            z_peak = config_.maximum_patch_height;
        }

        // project center of patch until it hits z_peak
        cv::Point2d uv = cv::Point2d(rect.x + rect.width/2, rect.y + rect.height/2) + patch_origin;
        cv::Point3d cvxyz = model.projectPixelTo3dRay(uv);
        tf::Stamped<tf::Vector3> patch_ray(
                tf::Vector3(
                    cvxyz.x,
                    cvxyz.y,
                    cvxyz.z),
                patches_msg->header.stamp,
                patches_msg->header.frame_id);
        tf::Stamped<tf::Vector3> clipping_ray;
        listener_.transformVector( clipping_frame_id_, patch_ray, clipping_ray);
        clipping_ray.normalize();
        double r = (z_peak - camera_origin.z())/clipping_ray.z();
        // finally, compute expected object position
        tf::Stamped<tf::Vector3> stamped_camera_origin(
                tf::Vector3(camera_origin.x(),
                    camera_origin.y(),
                    camera_origin.z()),
                patches_msg->header.stamp,
                clipping_frame_id_);

        tf::Vector3 object_position = stamped_camera_origin + r*clipping_ray;

        ROS_DEBUG_STREAM("patch_ray: (" <<
                patch_ray.x() << ", " << patch_ray.y() << ", " << patch_ray.z() << ") ");
        ROS_DEBUG_STREAM("clipping_ray: (" <<
                clipping_ray.x() << ", " << clipping_ray.y() << ", " << clipping_ray.z() << ") " << " z_peak: " << z_peak << " r: " << r);

        // put corresponding point_stamped in output message
        tf::Stamped<tf::Vector3> point(
                object_position,
                patches_msg->header.stamp,
                clipping_frame_id_);
        if(listener_.canTransform(output_frame_id_, point.frame_id_, point.stamp_))
        {
            tf::Stamped<tf::Vector3> output_point;
            listener_.transformPoint(output_frame_id_, point, output_point);
            tf::pointStampedTFToMsg(output_point, positioned_patch.world_point);
        }
        else
        {
            tf::pointStampedTFToMsg(point, positioned_patch.world_point);
        }
        positioned_patches.patch_array.push_back(positioned_patch);

        if(debug_marker_out.getNumSubscribers()>0)
        {
            visualization_msgs::Marker marker;
            marker.id = mid++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.header = positioned_patch.world_point.header;
            marker.pose.position = positioned_patch.world_point.point;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            vis_markers.markers.push_back(marker);
        }
    }

    patches_out.publish( positioned_patches );
    if(debug_marker_out.getNumSubscribers()>0)
    {
        debug_marker_out.publish(vis_markers);
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointCloudProjector");
    ros::NodeHandle nh;
    linemod_detector::PointCloudProjector proj(nh);
    ros::spin();
}

