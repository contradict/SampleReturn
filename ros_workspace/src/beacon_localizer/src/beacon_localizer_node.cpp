#include <vector>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace beacon_filter_node {
class BeaconKFNode
{
    public:
        BeaconKFNode( void );
        ~BeaconKFNode( void );
        void createFilter(void);
        void initializeRos( void );

    private:
        void getCovarianceMatrix(std::string param_name, MatrixWrapper::SymmetricMatrix& m);
        void transformBroadcastCallback( const ros::TimerEvent& e );
        void beaconCallback( geometry_msgs::PoseWithCovarianceStampedConstPtr msg );
        void filterUpdateCallback( const ros::TimerEvent& e );

        std::string _world_fixed_frame;
        std::string _odometry_frame;

        BFL::LinearAnalyticConditionalGaussian *_system_pdf;
        BFL::LinearAnalyticSystemModelGaussianUncertainty *_system_model;

        BFL::Gaussian *_prior;
        BFL::ExtendedKalmanFilter *_filter;

        ros::Subscriber          _odometry_sub;
        ros::Subscriber          _beacon_sub;
        tf::TransformListener    _tf;
        tf::TransformBroadcaster _tf_broadcast;
        ros::Timer               _transform_broadcast_timer;
        ros::Timer               _update_timer;

        std::string _camera_frame_id;
        ros::Time _last_beacon_time;

        tf::StampedTransform _T_odom;  //base_link in odom
        tf::StampedTransform _T_map_to_odom;
};

BeaconKFNode::BeaconKFNode( void ):
    _system_pdf(NULL),
    _system_model(NULL),
    _prior(NULL),
    _filter(NULL),
    _camera_frame_id(""),
    _last_beacon_time(ros::Time(0))
{
}

BeaconKFNode::~BeaconKFNode( void )
{
    if(_system_pdf)
        delete _system_pdf;
    if(_system_model)
        delete _system_model;
    if(_prior)
        delete _prior;
    if(_filter)
        delete _filter;
}

void BeaconKFNode::createFilter( void)
{
    MatrixWrapper::Matrix systemA(3,3);
    systemA(1,1) = 1.0; systemA(1,2) = 0.0; systemA(1,3) = 0.0;
    systemA(2,1) = 0.0; systemA(2,2) = 1.0; systemA(2,3) = 0.0;
    systemA(3,1) = 0.0; systemA(3,2) = 0.0; systemA(3,3) = 1.0;

    std::vector<MatrixWrapper::Matrix> systemMats(1);
    systemMats[0] = systemA;

    MatrixWrapper::SymmetricMatrix sysNoiseCovariance(3);
    getCovarianceMatrix("system_noise_covariance", sysNoiseCovariance);
    ROS_INFO_STREAM("system noise covariance: " << sysNoiseCovariance);
    MatrixWrapper::ColumnVector sysNoiseMu(3); // zeros
    sysNoiseMu(1) = 0.0;
    sysNoiseMu(2) = 0.0;
    sysNoiseMu(3) = 0.0;

    BFL::Gaussian sysUncertainty(sysNoiseMu, sysNoiseCovariance);

    _system_pdf = new BFL::LinearAnalyticConditionalGaussian(systemMats, sysUncertainty);

    _system_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(_system_pdf);


    MatrixWrapper::ColumnVector priorMu(3);
    priorMu(1) = 0.0;
    priorMu(2) = 0.0;
    priorMu(3) = 0.0;
    MatrixWrapper::SymmetricMatrix priorCov(3);
    priorCov(1,1) = 1.0; priorCov(1,2) = 0.0; priorCov(1,3) = 0.0;
                         priorCov(2,2) = 1.0; priorCov(2,3) = 0.0;
                                              priorCov(3,3) = 0.5;
    _prior = new BFL::Gaussian(priorMu, priorCov);

    _filter = new BFL::ExtendedKalmanFilter(_prior);

}

void BeaconKFNode::initializeRos( void )
{
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh("");

    private_nh.param("world_fixed_frame", _world_fixed_frame, std::string("map"));
    private_nh.param("odometry_frame", _odometry_frame, std::string("odom"));

    //beacon message subscriber callback
    _beacon_sub = nh.subscribe( "beacon_pose", 1, &BeaconKFNode::beaconCallback, this);

    //define timer rates
    double update_period;
    double broadcast_period;
    private_nh.param("update_period", update_period, 1.0);
    private_nh.param("broadcast_period", broadcast_period, 0.05);

    //timer for broadcasting the map correction xfrom    
    _transform_broadcast_timer = private_nh.createTimer(ros::Duration(broadcast_period),
                                           &BeaconKFNode::transformBroadcastCallback,
                                           this);
    
    //timer for updating the EKF
    _update_timer = private_nh.createTimer(ros::Duration(update_period),
                                           &BeaconKFNode::filterUpdateCallback,
                                           this);

}

void BeaconKFNode::transformBroadcastCallback( const ros::TimerEvent& e )
{
    (void) e; //so the compiler doesn't whine?
    
    ros::Time now = ros::Time::now();
    
    //ask the filter for the current state
    MatrixWrapper::ColumnVector state = _filter->PostGet()->ExpectedValueGet();
    tf::Vector3 odom_origin;
    tf::Quaternion odom_orientation;
    odom_orientation.setW(1.0);
    if( isnan(state(1)) || isnan(state(2)) || isnan(state(3)) ||
        isinf(state(1)) || isinf(state(2)) || isinf(state(3)) )
    {
        ROS_ERROR("NaN state");
    }
    else
    {
        odom_origin.setX( state(1) );
        odom_origin.setY( state(2) );
        odom_origin.setZ(0);
        odom_orientation = tf::createQuaternionFromYaw( state(3) );
    }
    
    //This is the map to odom transform, broadcast it and save it in a variable.
    //It will be used elsewhere in this class
    _T_map_to_odom = tf::StampedTransform(tf::Transform(odom_orientation, odom_origin),
                                          now,
                                          _world_fixed_frame,
                                          _odometry_frame);
                                          //"beacon_localizer_correction");
    _tf_broadcast.sendTransform( _T_map_to_odom );

}

void BeaconKFNode::beaconCallback( geometry_msgs::PoseWithCovarianceStampedConstPtr msg )
{
    ROS_INFO("beacon");
    tf::StampedTransform T_beacon_to_camera;
    tf::poseMsgToTF( msg->pose.pose, T_beacon_to_camera);
    ros::Time beacon_stamp = msg->header.stamp;
    T_beacon_to_camera.stamp_ = beacon_stamp;
    _camera_frame_id = msg->header.frame_id;

    tf::StampedTransform T_camera_to_base;
    _tf.lookupTransform("base_link", _camera_frame_id, beacon_stamp, T_camera_to_base);

    tf::StampedTransform T_base_to_odom;
    _tf.lookupTransform(_odometry_frame, "base_link", beacon_stamp, T_base_to_odom);            
            
    //this is in the reverse of the normal xform direction, for the error xform calc
    tf::StampedTransform T_map_to_beacon;
    _tf.lookupTransform("beacon", _world_fixed_frame, beacon_stamp, T_map_to_beacon);
            
    //This calculates the correction thusly:
    //map->odom->base->camera->measured-beacon * beacon->map
    tf::Transform T_map_to_odom = _T_map_to_odom*T_base_to_odom*T_camera_to_base*T_beacon_to_camera*T_map_to_beacon;
    
    //broadcast this T_map_to odom from real map
    tf::StampedTransform test_map_to_odom(T_map_to_odom,
                                          msg->header.stamp,
                                          _world_fixed_frame,
                                          "beacon_localizer_T");
    _tf_broadcast.sendTransform( test_map_to_odom );    

    MatrixWrapper::ColumnVector measurement(3);

    measurement(1) = T_map_to_odom.getOrigin()[0];
    measurement(2) = T_map_to_odom.getOrigin()[1];
    measurement(3) = tf::getYaw( T_map_to_odom.getRotation() );
    ROS_DEBUG_STREAM("measurement: " << measurement.transpose() );

    MatrixWrapper::ColumnVector state =_filter->PostGet()->ExpectedValueGet();
    MatrixWrapper::ColumnVector err = (state-measurement);
    double distance = sqrt(err.transpose()*err);

    MatrixWrapper::SymmetricMatrix cov(6);
    cov(1,1) = msg->pose.covariance[0]; cov(1,2) = msg->pose.covariance[1]; cov(1,3) = msg->pose.covariance[2]; cov(1,4) = msg->pose.covariance[3]; cov(1,5) = msg->pose.covariance[4]; cov(1,6) = msg->pose.covariance[5];
    cov(2,1) = msg->pose.covariance[6]; cov(2,2) = msg->pose.covariance[7]; cov(2,3) = msg->pose.covariance[8]; cov(2,4) = msg->pose.covariance[9]; cov(2,5) = msg->pose.covariance[10];cov(2,6) = msg->pose.covariance[11];
    cov(3,1) = msg->pose.covariance[12];cov(3,2) = msg->pose.covariance[13];cov(3,3) = msg->pose.covariance[14];cov(3,4) = msg->pose.covariance[15];cov(3,5) = msg->pose.covariance[16];cov(3,6) = msg->pose.covariance[17];
    cov(4,1) = msg->pose.covariance[18];cov(4,2) = msg->pose.covariance[19];cov(4,3) = msg->pose.covariance[20];cov(4,4) = msg->pose.covariance[21];cov(4,5) = msg->pose.covariance[22];cov(4,6) = msg->pose.covariance[23];
    cov(5,1) = msg->pose.covariance[24];cov(5,2) = msg->pose.covariance[25];cov(5,3) = msg->pose.covariance[26];cov(5,4) = msg->pose.covariance[27];cov(5,5) = msg->pose.covariance[28];cov(5,6) = msg->pose.covariance[29];
    cov(6,1) = msg->pose.covariance[30];cov(6,2) = msg->pose.covariance[31];cov(6,3) = msg->pose.covariance[32];cov(6,4) = msg->pose.covariance[33];cov(6,5) = msg->pose.covariance[34];cov(6,6) = msg->pose.covariance[35];

    /*
       tf::Matrix3x3 pointDevMat(
       sqrt(cov(1,1)),              0,             0,
       0.0, sqrt(cov(2,2)),             0,
       0.0,            0.0, sqrt(cov(3,3)));
       ROS_INFO("Point deviation");
       ROS_INFO("[ %f, %f, %f ]", pointDevMat[0][0], pointDevMat[0][1], pointDevMat[0][2]);
       ROS_INFO("[ %f, %f, %f ]", pointDevMat[1][0], pointDevMat[1][1], pointDevMat[1][2]);
       ROS_INFO("[ %f, %f, %f ]", pointDevMat[2][0], pointDevMat[2][1], pointDevMat[2][2]);

       tf::Matrix3x3 rpointDev = _T_odom.getBasis()*T_camera_to_base.getBasis()*pointDevMat*T_map_to_beacon.getBasis();
       tf::Matrix3x3 rpointCov = rpointDev*rpointDev;
       ROS_INFO("rPoint covariance");
       ROS_INFO("[ %f, %f, %f ]", rpointCov[0][0], rpointCov[0][1], rpointCov[0][2]);
       ROS_INFO("[ %f, %f, %f ]", rpointCov[1][0], rpointCov[1][1], rpointCov[1][2]);
       ROS_INFO("[ %f, %f, %f ]", rpointCov[2][0], rpointCov[2][1], rpointCov[2][2]);
       */


    MatrixWrapper::SymmetricMatrix measNoiseCovariance(3);
    //measNoiseCovariance(1,1) = rpointCov[0][0]; measNoiseCovariance(1,2) = rpointCov[0][1]; measNoiseCovariance(1,3) = 0;
    //measNoiseCovariance(2,1) = rpointCov[1][0]; measNoiseCovariance(2,2) = rpointCov[1][1]; measNoiseCovariance(2,3) = 0;
    //measNoiseCovariance(3,1) = 0;               measNoiseCovariance(3,2) = 0;               measNoiseCovariance(3,3) = cov(5,5);
    measNoiseCovariance(1,1) = cov(3,3); measNoiseCovariance(1,2) =      0.0; measNoiseCovariance(1,3) = 0.0;
    measNoiseCovariance(2,1) =      0.0; measNoiseCovariance(2,2) = cov(3,3); measNoiseCovariance(2,3) = 0.0;
    measNoiseCovariance(3,1) =      0.0; measNoiseCovariance(3,2) =      0.0; measNoiseCovariance(3,3) = cov(5,5);
    ROS_INFO_STREAM("measurement noise covariance: " << measNoiseCovariance);
    MatrixWrapper::ColumnVector measNoiseMu(3); // zeros
    measNoiseMu(1) = 0.0;
    measNoiseMu(2) = 0.0;
    measNoiseMu(3) = 0.0;
    BFL::Gaussian measUncertainty(measNoiseMu, measNoiseCovariance);

    MatrixWrapper::Matrix measH(3,3);
    measH(1,1) = 1.0; measH(1,2) = 0.0; measH(1,3) = 0.0;
    measH(2,1) = 0.0; measH(2,2) = 1.0; measH(2,3) = 0.0;
    measH(3,1) = 0.0; measH(3,2) = 0.0; measH(3,3) = 1.0;

    BFL::LinearAnalyticConditionalGaussian beaconMeasurementPdf(measH, measUncertainty);

    BFL::LinearAnalyticMeasurementModelGaussianUncertainty beaconMeasurementModel(&beaconMeasurementPdf);


    _filter->Update(_system_model, &beaconMeasurementModel, measurement);

    ROS_DEBUG_STREAM("State: " << _filter->PostGet()->ExpectedValueGet().transpose() );
    ROS_DEBUG_STREAM("Covariance: " << _filter->PostGet()->CovarianceGet() );
}

void BeaconKFNode::filterUpdateCallback( const ros::TimerEvent& e )
{
    (void) e;
    ROS_DEBUG_STREAM("pre-Update state: " << _filter->PostGet()->ExpectedValueGet().transpose() );
    ROS_DEBUG_STREAM("Covariance: " << _filter->PostGet()->CovarianceGet() );
    _filter->Update(_system_model);
    ROS_INFO_STREAM("Update state: " << _filter->PostGet()->ExpectedValueGet().transpose() );
    ROS_INFO_STREAM("Covariance: " << _filter->PostGet()->CovarianceGet() );
}

void BeaconKFNode::getCovarianceMatrix(std::string param_name, MatrixWrapper::SymmetricMatrix& m)
{
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue values;
    private_nh.getParam(param_name, values);
    if( values.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
        ROS_ERROR("Unable to read covariance %s, not an array", param_name.c_str());
        return;
    }
    if( values.size() < 6 )
    {
        ROS_ERROR("Unable to read covariance %s, array too short: %d", param_name.c_str(), values.size());
        return;
    }
    int i=0;
    for (uint32_t row = 1; row <= m.rows(); ++row)
    {
        for (uint32_t column = row; column <= m.columns(); ++column)
        {
            double x;
            if(i>=values.size())
            {
                ROS_ERROR("Need at least 6 values, have %d", i);
                return;
            }
            XmlRpc::XmlRpcValue value=values[i++];
            if( value.getType() == XmlRpc::XmlRpcValue::TypeInt )
            {
                x=int(value);
            }
            else if(value.getType() == XmlRpc::XmlRpcValue::TypeDouble )
            {
                x=double(value);
            }
            else
            {
                std::string vstr = value;
                ROS_ERROR("Unable to read covariance matrix %s, value at %d is not a number: %s",
                        param_name.c_str(), i, vstr.c_str());
                return;
            }
            m(row, column) = x;
        }
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "beacon_localizer");
    beacon_filter_node::BeaconKFNode node;
    node.createFilter();
    node.initializeRos();
    ros::spin();

    return 0;
}
