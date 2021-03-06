#include <vector>
#include <ros/ros.h>
#include <math.h>

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

#include <dynamic_reconfigure/server.h>
#undef DEFAULT //bfl has a macro named DEFAULT, dynamic_reconfigure tries to use the same name
#include <beacon_localizer/beacon_localizer_paramsConfig.h>

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
        void configCallback( beacon_localizer::beacon_localizer_paramsConfig &config, uint32_t level );
        
        std::string _world_fixed_frame;
        std::string _odometry_frame;
        std::string _platform_frame;

        std::vector<double> _platform_x_coords;
        std::vector<double> _platform_y_coords;
        tf::Vector3 _platform_origin;
        tf::Quaternion _platform_orientation;
        std::vector<double> _reference_coords;
        
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
        tf::StampedTransform _T_map_to_platform;

        dynamic_reconfigure::Server<beacon_localizer::beacon_localizer_paramsConfig> dr_server;
};

BeaconKFNode::BeaconKFNode( void ):
    _system_pdf(NULL),
    _system_model(NULL),
    _prior(NULL),
    _filter(NULL),
    _tf(ros::Duration(30.0)),
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
    private_nh.param("platform_frame", _platform_frame, std::string("platform"));

    //setup platform offset position and angle
    int platform_index;
    std::vector<double> empty_vec;    
    private_nh.param("platform_index", platform_index, 0);
    private_nh.param("platform_x_coords", _platform_x_coords, empty_vec);
    private_nh.param("platform_y_coords", _platform_y_coords, empty_vec); 

    double reference_x;
    double reference_y;
    private_nh.param("reference_x", reference_x, 1.0);
    private_nh.param("reference_y", reference_y, 0.0);
    _reference_coords.push_back(reference_x);
    _reference_coords.push_back(reference_y);
    
    double platform_angle;
    private_nh.param("platform_orientation", platform_angle, 0.0);
    _platform_origin.setX(_platform_x_coords[platform_index]);
    _platform_origin.setY(_platform_y_coords[platform_index]);
    _platform_origin.setZ(0);
    platform_angle = platform_angle*(M_PI/180);
    platform_angle = platform_angle + atan(_reference_coords[1]/_reference_coords[0]);
    _platform_orientation = tf::createQuaternionFromYaw(platform_angle);
    
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

    //set dynamic reconfigure callback 
    dr_server.setCallback(boost::bind(&BeaconKFNode::configCallback, this,  _1, _2));
    
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
        ROS_ERROR("BEACON LOCALIZER: NaN state");
    }
    else
    {
        odom_origin.setX( state(1) );
        odom_origin.setY( state(2) );
        odom_origin.setZ(0);
        odom_orientation = tf::createQuaternionFromYaw( state(3) );
    }
    
    std::vector<tf::StampedTransform> _transforms;
    
    //This is the map to odom transform, broadcast it and save it in a variable.
    _T_map_to_odom = tf::StampedTransform(tf::Transform(odom_orientation, odom_origin),
                                          now,
                                          _world_fixed_frame,
                                          _odometry_frame);
                                          //"beacon_localizer_correction");
    _transforms.push_back(_T_map_to_odom);
    
    //this is the platform to map transform
    _T_map_to_platform = tf::StampedTransform(tf::Transform(_platform_orientation,
                                                            _platform_origin),
                                              now,
                                              _world_fixed_frame,
                                              _platform_frame);
    _transforms.push_back(_T_map_to_platform);
        
    _tf_broadcast.sendTransform(_transforms);
        
}

void BeaconKFNode::beaconCallback( geometry_msgs::PoseWithCovarianceStampedConstPtr msg )
{
    tf::StampedTransform T_beacon_to_camera;
    tf::poseMsgToTF( msg->pose.pose, T_beacon_to_camera);
    ros::Time beacon_stamp = msg->header.stamp;
    T_beacon_to_camera.stamp_ = beacon_stamp;
    _camera_frame_id = msg->header.frame_id;
    
    //measured map->odom transform
    tf::Transform T_measured_mto;
    
    try {
        _tf.waitForTransform("base_link", _camera_frame_id,
                             beacon_stamp, ros::Duration(1.0));
        
        tf::StampedTransform T_camera_to_base;
        _tf.lookupTransform("base_link", _camera_frame_id, beacon_stamp, T_camera_to_base);
    
        _tf.waitForTransform(_odometry_frame, "base_link",
                             beacon_stamp, ros::Duration(1.0));
    
        tf::StampedTransform T_base_to_odom;
        _tf.lookupTransform(_odometry_frame, "base_link", beacon_stamp, T_base_to_odom);            
                
        //this is in the reverse of the normal xform direction, for the error xform calc
        //this is a static xform from beacon_finder, timestamp isn't important
        tf::StampedTransform T_map_to_beacon;
        _tf.lookupTransform("beacon", _world_fixed_frame, ros::Time(0), T_map_to_beacon);
                
        //This calculates the correction thusly:
        //odom->base->camera->measured-beacon * beacon->map
        T_measured_mto = T_base_to_odom*T_camera_to_base*T_beacon_to_camera*T_map_to_beacon;
        T_measured_mto = T_measured_mto.inverse();
        
        //broadcast this T_map_to odom from real map
        tf::StampedTransform test_map_to_odom(T_measured_mto,
                                              msg->header.stamp,
                                              _world_fixed_frame,
                                              "beacon_localizer_T");
        _tf_broadcast.sendTransform( test_map_to_odom );
    }
    catch (tf::TransformException &ex) {
         ROS_ERROR("BEACON_LOCALIZER transform lookup failure: %s", ex.what());
         return;
    }

    MatrixWrapper::ColumnVector measurement(3);

    measurement(1) = T_measured_mto.getOrigin()[0];
    measurement(2) = T_measured_mto.getOrigin()[1];
    measurement(3) = tf::getYaw( T_measured_mto.getRotation() );
    ROS_DEBUG_STREAM("BEACON LOCALIZER measurement: " << measurement.transpose() );

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
    ROS_DEBUG_STREAM("BEACON LOCALIZER measurement noise covariance: " << measNoiseCovariance);
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

    ROS_DEBUG_STREAM("BEACON LOCALIZER State: " << _filter->PostGet()->ExpectedValueGet().transpose() );
    ROS_DEBUG_STREAM("BEACON LOCALIZER Covariance: " << _filter->PostGet()->CovarianceGet() );
}

void BeaconKFNode::filterUpdateCallback( const ros::TimerEvent& e )
{
    (void) e;
    ROS_DEBUG_STREAM("BEACON LOCALIZER Pre-update state: " << _filter->PostGet()->ExpectedValueGet().transpose() );
    ROS_DEBUG_STREAM("BEACON LOCALIZER Covariance: " << _filter->PostGet()->CovarianceGet() );
    _filter->Update(_system_model);
    ROS_DEBUG_STREAM("BEACON LOCALIZER Update state: " << _filter->PostGet()->ExpectedValueGet().transpose() );
    ROS_DEBUG_STREAM("BEACON LOCALIZER Covariance: " << _filter->PostGet()->CovarianceGet() );
}

/* Dynamic reconfigure callback */
void BeaconKFNode::configCallback(beacon_localizer::beacon_localizer_paramsConfig &config, uint32_t level)
{
    double platform_angle;
    int platform_index;
    
    _reference_coords[0] = config.reference_x;
    _reference_coords[1] = config.reference_y;

    //get platform orientation in degrees, convert to reference system
    platform_angle =  config.platform_orientation;
    platform_angle = platform_angle*(M_PI/180);
    platform_angle = platform_angle + atan(_reference_coords[1]/_reference_coords[0]);
    _platform_orientation = tf::createQuaternionFromYaw(platform_angle);

    platform_index = config.platform_index;
    _platform_origin.setX(_platform_x_coords[platform_index]);
    _platform_origin.setY(_platform_y_coords[platform_index]);
    
}

void BeaconKFNode::getCovarianceMatrix(std::string param_name, MatrixWrapper::SymmetricMatrix& m)
{
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue values;
    private_nh.getParam(param_name, values);
    if( values.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
        ROS_ERROR("BEACON LOCALIZER Unable to read covariance %s, not an array", param_name.c_str());
        return;
    }
    if( values.size() < 6 )
    {
        ROS_ERROR("BEACON LOCALIZER Unable to read covariance %s, array too short: %d", param_name.c_str(), values.size());
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
                ROS_ERROR("BEACON LOCALIZER Need at least 6 values, have %d", i);
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
                ROS_ERROR("BEACON LOCALIZER Unable to read covariance matrix %s, value at %d is not a number: %s",
                        param_name.c_str(), i, vstr.c_str());
                return;
            }
            m(row, column) = x;
        }
    }
}

} //end name_space

int main(int argc, char **argv)
{
    ros::init(argc, argv, "beacon_localizer");
    beacon_filter_node::BeaconKFNode node;
    node.createFilter();
    node.initializeRos();
    ros::spin();

    return 0;
}
