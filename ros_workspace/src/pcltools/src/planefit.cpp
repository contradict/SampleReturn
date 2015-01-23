#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pcltools/PlaneFitConfig.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_listener.h>

namespace pcltools {
class PCLPlaneFit {
    public:
        PCLPlaneFit();
        ~PCLPlaneFit();

    private:
        ros::Subscriber points_sub_;
        ros::Publisher coefficient_pub_;
        ros::Publisher marker_pub_;
        ros::Publisher inlier_pub_;

        tf::TransformListener *listener;

        boost::recursive_mutex reconfigure_mutex_;
        dynamic_reconfigure::Server<pcltools::PlaneFitConfig> reconfigure_server;

        size_t min_inliers_;
        std::string input_frame_, output_frame_;

        pcl::SACSegmentation<pcl::PointXYZ> seg_;
        void pointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
        void reconfigureCallback(pcltools::PlaneFitConfig &config, uint32_t level);

};

PCLPlaneFit::PCLPlaneFit() :
    reconfigure_server(reconfigure_mutex_)
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    points_sub_ = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("points", 1,
            &PCLPlaneFit::pointsCallback, this);
    coefficient_pub_ = nh.advertise<pcl_msgs::ModelCoefficients>("plane_coefficients", 10);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("plane_normal", 10);
    inlier_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("inlier", 10);

    listener = new tf::TransformListener();
    reconfigure_server.setCallback( boost::bind(&PCLPlaneFit::reconfigureCallback, this, _1, _2) );

    XmlRpc::XmlRpcValue axis_param;
    pnh.getParam ("axis", axis_param);
    Eigen::Vector3f axis = Eigen::Vector3f::Zero ();
    if(axis_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        if(axis_param.size () != 3)
        {
            ROS_ERROR (" Parameter 'axis' given but with a different number of values (%d) than required (3)!", axis_param.size ());
        }
        else
        {
            for (int i = 0; i < 3; ++i)
            {
                if (axis_param[i].getType () != XmlRpc::XmlRpcValue::TypeDouble)
                {
                    ROS_ERROR (" Need floating point values for 'axis' parameter.");
                    break;
                }
                double value = axis_param[i]; axis[i] = value;
            }
        }
    }

    pnh.param( "input_frame", input_frame_, std::string(""));

    seg_.setAxis(axis);
    ROS_INFO_STREAM("Set axis: " << axis.transpose() );
    pcltools::PlaneFitConfig current_config;
    reconfigure_server.getConfigDefault( current_config );
    current_config.x = axis.x();
    current_config.y = axis.y();
    current_config.z = axis.z();
    reconfigure_server.updateConfig( current_config );
}

PCLPlaneFit::~PCLPlaneFit()
{
}

void
PCLPlaneFit::reconfigureCallback(pcltools::PlaneFitConfig &config, uint32_t level)
{
    (void)level;
    seg_.setModelType( config.modelType );
    seg_.setMethodType( config.methodType );
    seg_.setMaxIterations( config.max_iterations );
    seg_.setProbability( config.probability );
    seg_.setDistanceThreshold( config.distance_threshold );
    seg_.setOptimizeCoefficients( config.optimize_coefficients );
    seg_.setRadiusLimits( config.radius_min, config.radius_max );
    seg_.setEpsAngle( config.eps_angle );

    Eigen::Vector3f axis;
    axis << config.x, config.y, config.z;
    seg_.setAxis( axis );

    min_inliers_ = config.min_inliers;
    input_frame_ = config.input_frame;
    output_frame_ = config.output_frame;

}

void
PCLPlaneFit::pointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ptsin)
{
    boost::recursive_mutex::scoped_lock lock(reconfigure_mutex_);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane( new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_xformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptsptr;

    if( input_frame_.length() > 0 )
    {
        if(!listener->waitForTransform( input_frame_, ptsin->header.frame_id.c_str(),
                    pcl_conversions::fromPCL(ptsin->header.stamp), ros::Duration(0.1)))
        {
            ROS_ERROR( "Unable to wait for up transform %s->%s",
                    ptsin->header.frame_id.c_str(), input_frame_.c_str());
            return;
        }
        try
        {
            pcl_ros::transformPointCloud( input_frame_, *ptsin, *pts_xformed, *listener);
        }
        catch( tf::TransformException ex)
        {
            ROS_ERROR( "Unable to look up transform %s->%s: %s",
                    ptsin->header.frame_id.c_str(), input_frame_.c_str(), ex.what());
            return;
        }
        ptsptr = pts_xformed;
    }
    else
    {
        ptsptr = ptsin;
    }

    seg_.setInputCloud(ptsptr);
    seg_.segment(*plane, *coefficients);

    if(plane->indices.size() < min_inliers_)
    {
        ROS_ERROR("No plane model found");
        return;
    }
    pcl_msgs::ModelCoefficients coeffs;
    coeffs.header.frame_id = ptsptr->header.frame_id;
    coeffs.header.seq = ptsptr->header.seq;
    pcl_conversions::fromPCL(ptsptr->header.stamp, coeffs.header.stamp);
    coeffs.values = coefficients->values;
    coefficient_pub_.publish( coeffs );

    pcl::PointCloud<pcl::PointXYZ>::Ptr inl(new pcl::PointCloud<pcl::PointXYZ>);
    inl->header = ptsptr->header;
    inl->width = 1;
    inl->height = plane->indices.size();
    Eigen::Vector3d centroid;
    centroid.setZero();
    for(size_t i = 0; i<plane->indices.size(); i++)
    {
        inl->points.push_back( ptsptr->points[plane->indices[i]] );
        centroid += ptsptr->points[plane->indices[i]].getVector3fMap().cast<double>();
    }
    inlier_pub_.publish( inl );

    centroid /= plane->indices.size();
    visualization_msgs::Marker mark;
    mark.header = coeffs.header;
    mark.ns = "plane";
    mark.id = 0;
    mark.type = visualization_msgs::Marker::ARROW;
    mark.action = visualization_msgs::Marker::ADD;
    tf::pointEigenToMsg( centroid, mark.pose.position );
    Eigen::Quaterniond orientation;
    Eigen::Vector3d xhat;
    xhat.setZero();
    xhat[0] = 1.0;
    Eigen::Vector3d plane_normal;
    plane_normal << coefficients->values[0],
                 coefficients->values[1],
                 coefficients->values[2];
    orientation.setFromTwoVectors(xhat, plane_normal);
    tf::quaternionEigenToMsg( orientation, mark.pose.orientation);
    mark.scale.x = 1.0;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    mark.color.a = 1.0;
    mark.color.r = 0.0;
    mark.color.g = 0.5;
    mark.color.b = 1.0;
    marker_pub_.publish( mark );

}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planefit");

    pcltools::PCLPlaneFit fit;

    ros::spin();
}
