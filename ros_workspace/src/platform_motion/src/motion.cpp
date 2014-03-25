#include <math.h>
#include <unistd.h>
#include <tr1/memory>
#include <poll.h>
#include <algorithm>

#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion_msgs/HomeAction.h>
#include <platform_motion_msgs/Enable.h>
#include <platform_motion_msgs/SelectCommandSource.h>
#include <platform_motion/PlatformParametersConfig.h>
#include <platform_motion_msgs/GPIO.h>
#include <platform_motion_msgs/ServoStatus.h>
#include <platform_motion_msgs/Path.h>
#include <motion/wheelpod.h>

#include <motion/motion.h>

#define POLL_TIMEOUT_MS 10

namespace platform_motion {

Motion::Motion() :
    param_nh("~"),
    home_pods_action_server(nh_, "home_wheel_pods", false),
    home_carousel_action_server(nh_, "home_carousel", false),
    command_source(COMMAND_SOURCE_NONE),
    CAN_fd(-1),
    gpio_enabled(false),
    carousel_setup(false),
    pods_enabled(false),
    desired_pod_state(false),
    carousel_enabled(false),
    desired_carousel_state(false),
    pv_counter(0),
    joint_seq(0),
    m_debugPrintCounter(0)
{
    lastSegmentSent.port.steeringAngle = 0.0;
    lastSegmentSent.port.steeringVelocity = 0.0;
    lastSegmentSent.port.wheelDistance = 0.0;
    lastSegmentSent.port.wheelVelocity = 0.0;
    lastSegmentSent.port.duration = 0.25;
    lastSegmentSent.starboard.steeringAngle = 0.0;
    lastSegmentSent.starboard.steeringVelocity = 0.0;
    lastSegmentSent.starboard.wheelDistance = 0.0;
    lastSegmentSent.starboard.wheelVelocity = 0.0;
    lastSegmentSent.starboard.duration = 0.25;
    lastSegmentSent.stern.steeringAngle = 0.0;
    lastSegmentSent.stern.steeringVelocity = 0.0;
    lastSegmentSent.stern.wheelDistance = 0.0;
    lastSegmentSent.stern.wheelVelocity = 0.0;
    lastSegmentSent.stern.duration = 0.25;
    lastSegmentSent.time = ros::Time::now();

    ROS_INFO("Started in namespace %s", nh_.getNamespace().c_str());

    ROS_INFO("param in namespace %s", param_nh.getNamespace().c_str());

    param_nh.param("wheel_diameter", wheel_diameter, 0.330);
    param_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

    param_nh.param("center_pt_x", center_pt_x, 0.0);
    param_nh.param("center_pt_y", center_pt_y, 0.0);

    param_nh.param("min_wheel_speed", min_wheel_speed, 0.0005);

    param_nh.param("enable_wait_timeout_ms", enable_wait_timeout, 500);

    body_pt << center_pt_x, center_pt_y;
    enable_pods_server = nh_.advertiseService("enable_wheel_pods",
            &Motion::enableWheelPodsCallback, this);
    enable_carousel_server = nh_.advertiseService("enable_carousel",
            &Motion::enableCarouselCallback, this);

    select_command_server = nh_.advertiseService("CAN_select_command_source",
            &Motion::selectCommandSourceCallback, this);
    planner_sub = nh_.subscribe("planner_command", 2, &Motion::plannerTwistCallback, this);
    joystick_sub = nh_.subscribe("joystick_command", 2, &Motion::joystickTwistCallback, this);
    servo_sub = nh_.subscribe("CAN_servo_command", 2, &Motion::servoTwistCallback, this);

    carousel_sub = nh_.subscribe("carousel_angle", 2, &Motion::carouselCallback,
            this);

    gpio_sub = nh_.subscribe("gpio_write", 2, &Motion::gpioSubscriptionCallback,
            this);

    nh_.subscribe("planned_path", 1, &Motion::plannedPathCallback, this);

    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("platform_joint_state", 1);

    gpio_pub = nh_.advertise<platform_motion_msgs::GPIO>("gpio_read", 1);

    battery_voltage_pub = nh_.advertise<std_msgs::Float64>("battery_voltage", 1);

    status_pub = nh_.advertise<platform_motion_msgs::ServoStatus>("CAN_status_word", 10);


    home_pods_action_server.registerGoalCallback(boost::bind(&Motion::doHomePods, this));
    home_carousel_action_server.registerGoalCallback(boost::bind(&Motion::doHomeCarousel, this));

    reconfigure_server.setCallback(boost::bind(&Motion::reconfigureCallback, this, _1, _2));

    double status_publish_interval;
    param_nh.param("status_publish_interval", status_publish_interval, 60.0);
    status_publish_timer = nh_.createTimer(ros::Duration(status_publish_interval), &Motion::statusPublishCallback, this);
}

bool Motion::openBus(void)
{
    int CAN_channel, CAN_baud;
 
    param_nh.param("CAN_channel", CAN_channel, 0);
    nh_.param("CAN_baud", CAN_baud, 1000000);

    std::tr1::shared_ptr<CANOpen::KvaserInterface> pintf(new CANOpen::KvaserInterface(
                                                CAN_channel,
                                                CAN_baud
                                               ));

    CAN_fd = pintf->getFD();

    if(CAN_fd<0) {
        return false;
    }

    ROS_INFO("Opened bus, fd %d", CAN_fd);

    pbus = std::tr1::shared_ptr<CANOpen::Bus>(new CANOpen::Bus(pintf, false));

    std::tr1::shared_ptr<CANOpen::SYNC> psync(new CANOpen::SYNC(0,
                CANOpen::SYNCCallbackObject(
                static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::SYNCCallbackObject::CallbackFunction>(&Motion::syncCallback))));

    pbus->add(psync);

    return true;
}

void Motion::createServos()
{
    int port_steering_id, port_wheel_id;
    double port_steering_min, port_steering_max, port_steering_offset;
    int starboard_steering_id, starboard_wheel_id;
    double starboard_steering_min, starboard_steering_max, starboard_steering_offset;
    int stern_steering_id, stern_wheel_id;
    double stern_steering_min, stern_steering_max, stern_steering_offset;
    int carousel_id;
    int sync_interval;
    int steering_encoder_counts, wheel_encoder_counts;
    int large_steering_move;

    CANOpen::CopleyServo::InputChangeCallback
        gpiocb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::CopleyServo::InputChangeCallback::CallbackFunction>(&Motion::gpioCallback));

    CANOpen::DS301CallbackObject pvcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::pvCallback));

    CANOpen::DS301CallbackObject stcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::statusCallback));

    CANOpen::DS301CallbackObject moreDataCb(
            static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(
                &Motion::moreDataNeededCallback)
            );

    // set the more data sent flag to the correct value (no data yet sent)
    this->moreDataSent = false;

    CANOpen::DS301CallbackObject errorCb(
            static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(
                &Motion::errorCallback)
            );

    param_nh.param("steering_encoder_counts", steering_encoder_counts, 4*2500);
    param_nh.param("wheel_encoder_counts", wheel_encoder_counts, 4*2500);
    param_nh.param("large_steering_move", large_steering_move, 30);

    param_nh.getParam("port_steering_id", port_steering_id);
    param_nh.getParam("port_wheel_id", port_wheel_id);
    param_nh.param("port_steering_min", port_steering_min, -M_PI_2);
    param_nh.param("port_steering_max", port_steering_max,  M_PI_2);
    param_nh.param("port_steering_offset", port_steering_offset,  0.0);
    port = std::tr1::shared_ptr<WheelPod>(new WheelPod(
                 pbus,
                 port_steering_id,
                 port_wheel_id,
                 port_steering_min,
                 port_steering_max,
                 port_steering_offset,
                 steering_encoder_counts,
                 wheel_encoder_counts,
                 large_steering_move,
                 wheel_diameter
                ));
    port->setCallbacks(pvcb, pvcb, gpiocb, moreDataCb, moreDataCb);
    port->steering.setStatusCallback(stcb);
    port->wheel.setStatusCallback(stcb);
    port->steering.setErrorCallback(errorCb);
    port->wheel.setErrorCallback(errorCb);

    param_nh.getParam("starboard_steering_id", starboard_steering_id);
    param_nh.getParam("starboard_wheel_id", starboard_wheel_id);
    param_nh.param("starboard_steering_min", starboard_steering_min, -M_PI_2);
    param_nh.param("starboard_steering_max", starboard_steering_max,  M_PI_2);
    param_nh.param("starboard_steering_offset", starboard_steering_offset,  0.0);
    starboard = std::tr1::shared_ptr<WheelPod>(new WheelPod(
                 pbus,
                 starboard_steering_id,
                 starboard_wheel_id,
                 starboard_steering_min,
                 starboard_steering_max,
                 starboard_steering_offset,
                 steering_encoder_counts,
                 wheel_encoder_counts,
                 large_steering_move,
                 wheel_diameter
                ));
    starboard->setCallbacks(pvcb, pvcb, gpiocb, moreDataCb, moreDataCb);
    starboard->steering.setStatusCallback(stcb);
    starboard->wheel.setStatusCallback(stcb);
    starboard->steering.setErrorCallback(errorCb);
    starboard->wheel.setErrorCallback(errorCb);

    // starboard is mounted backwards on the robot relative to the normal wheel velocity.
    starboard->setIsBackwards(true);

    param_nh.getParam("stern_steering_id", stern_steering_id);
    param_nh.getParam("stern_wheel_id", stern_wheel_id);
    param_nh.param("stern_steering_min", stern_steering_min, -M_PI_2);
    param_nh.param("stern_steering_max", stern_steering_max,  M_PI_2);
    param_nh.param("stern_steering_offset", stern_steering_offset,  0.0);
    stern = std::tr1::shared_ptr<WheelPod>(new WheelPod(
                 pbus,
                 stern_steering_id,
                 stern_wheel_id,
                 stern_steering_min,
                 stern_steering_max,
                 stern_steering_offset,
                 steering_encoder_counts,
                 wheel_encoder_counts,
                 large_steering_move,
                 wheel_diameter
                ));
    stern->setCallbacks(pvcb, pvcb, gpiocb, moreDataCb, moreDataCb);
    stern->steering.setStatusCallback(stcb);
    stern->wheel.setStatusCallback(stcb);
    stern->steering.setErrorCallback(errorCb);
    stern->wheel.setErrorCallback(errorCb);

    // stern is mounted backwards on the robot relative to the normal wheel velocity.
    stern->setIsBackwards(true);

    param_nh.getParam("carousel_id", carousel_id);
    param_nh.param("carousel_encoder_counts", carousel_encoder_counts, 4*2500);
    param_nh.param("carousel_offset", carousel_offset, 0.0);
    param_nh.param("sync_interval", sync_interval, 50000);
    param_nh.param("carousel_jerk_limit", carousel_jerk_limit, 5.0); // rev/s^3
    param_nh.param("carousel_profile_velocity", carousel_profile_velocity, 0.25); // rev/s
    param_nh.param("carousel_profile_acceleration", carousel_profile_acceleration, 0.3); // rev/s^2
    carousel = std::tr1::shared_ptr<CANOpen::CopleyServo>(new
            CANOpen::CopleyServo(carousel_id, sync_interval, pbus));
    carousel->setInputCallback(gpiocb);
    carousel->setPVCallback(pvcb);
    carousel->setStatusCallback(stcb);

    port->initialize();
    starboard->initialize();
    stern->initialize();
    carousel->initialize();
}

Motion::~Motion()
{
    CAN_thread_run = false;
    CAN_thread.join();
}

void Motion::shutdown(void)
{
    if(CAN_fd < 0)
        return;
    enable_pods_count = 6;
    enable_pods(false);
    boost::system_time now=boost::get_system_time();
    boost::unique_lock<boost::mutex> pod_lock(enable_pods_mutex);
    while( pods_enabled == true )
        enable_pods_cond.timed_wait(pod_lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    enable_carousel(false);
    boost::unique_lock<boost::mutex> carousel_lock(enable_carousel_mutex);
    while( carousel_enabled == true )
        enable_carousel_cond.timed_wait(carousel_lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    carousel->output(0x0, 0x7);
}

void Motion::runBus(void)
{
    struct pollfd pfd;
    int ret=0;
    CAN_thread_run=true;
    boost::unique_lock<boost::mutex> open_lock(CAN_mutex);
    while(!openBus() && CAN_thread_run ) {
        ROS_ERROR("Could not open CAN bus, trying again in 5 seconds");
        ros::Duration(5.0).sleep();
        if( ! ros::ok() ) {
            CAN_thread_run = false;
            break;
        }
    }
    open_lock.unlock();
    if( ! CAN_thread_run )
        return;

    createServos();

    pfd.fd = CAN_fd;
    pfd.events = POLLIN | POLLOUT;
    while(ret >= 0 && CAN_thread_run ){
        ret = poll(&pfd, 1, POLL_TIMEOUT_MS);
        if(ret>0) {
            if(pfd.revents) {
                bool canRead = pfd.revents&POLLIN;
                bool canWrite = pfd.revents&POLLOUT;
                boost::unique_lock<boost::mutex> run_lock(CAN_mutex);
                pbus->runonce(canRead, canWrite);
                run_lock.unlock();
                pfd.events |= POLLIN;
                if(canWrite) {
                    pfd.events |= POLLOUT;
                } else {
                    pfd.events &= ~POLLOUT;
                }
            }
        }
    }
}
 
void Motion::start(void)
{
    ROS_INFO("Start CAN bus thread");
    CAN_thread = boost::thread(boost::bind(&Motion::runBus, this));
    ROS_INFO("Start home pods server");
    home_pods_action_server.start();
    ROS_INFO("Start home carousel server");
    home_carousel_action_server.start();
    ROS_INFO("Waiting for transforms");
    ros::Time current = ros::Time(0);
    ros::Duration wait(1.0);
    while( !(
             listener.waitForTransform(child_frame_id, std::string("port_suspension"), current, wait) && 
             listener.waitForTransform(child_frame_id, std::string("starboard_suspension"), current, wait) &&
             listener.waitForTransform(child_frame_id, std::string("stern_suspension"), current, wait)
            ) && ros::ok()
         )
        ROS_INFO("Still Waiting for transforms");
    if( ros::ok() )
        ROS_INFO("Got all transforms");
}

void Motion::plannerTwistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    if(command_source == COMMAND_SOURCE_PLANNER)
        handleTwist(twist);
}

void Motion::joystickTwistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    if(command_source == COMMAND_SOURCE_JOYSTICK)
        handleTwist(twist);
}

void Motion::servoTwistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    if(command_source == COMMAND_SOURCE_SERVO)
        handleTwist(twist);
}

bool Motion::selectCommandSourceCallback(platform_motion_msgs::SelectCommandSource::Request &req,
                                          platform_motion_msgs::SelectCommandSource::Response &resp)
{
    resp.valid=true;
    resp.source=req.source;
    if(req.source == std::string("Joystick"))
        command_source=COMMAND_SOURCE_JOYSTICK;
    else if(req.source == std::string("Planner"))
        command_source=COMMAND_SOURCE_PLANNER;
    else if(req.source == std::string("Servo"))
        command_source=COMMAND_SOURCE_SERVO;
    else if(req.source == std::string("None"))
    {
        command_source=COMMAND_SOURCE_NONE;
        geometry_msgs::Twist *twist = new geometry_msgs::Twist();
        twist->linear.x=0;
        twist->linear.y=0;
        twist->angular.z=0;
        handleTwist(geometry_msgs::Twist::ConstPtr(twist));
    }
    else
    {
        ROS_ERROR("Unrecognized command source %s", req.source.c_str());
        switch(command_source)
        {
            case COMMAND_SOURCE_PLANNER:
                resp.source="Planner";
                break;
            case COMMAND_SOURCE_JOYSTICK:
                resp.source="Joystick";
                break;
            case COMMAND_SOURCE_SERVO:
                resp.source="Servo";
                break;
            case COMMAND_SOURCE_NONE:
                resp.source="None";
                break;
        }
        resp.valid=false;
    }
    ROS_INFO("Selected command source %s", req.source.c_str());
    return true;
}

void Motion::handleTwist(const geometry_msgs::Twist::ConstPtr twist)
{
    Eigen::Vector2d body_vel( twist->linear.x, twist->linear.y );
    double body_omega = twist->angular.z;
    double stern_wheel_speed, starboard_wheel_speed, port_wheel_speed;
    double stern_steering_angle, starboard_steering_angle, port_steering_angle;


    stern->getPosition(&stern_steering_angle, NULL, NULL, NULL);
    if(0>computePod(body_vel, body_omega, body_pt, "stern_suspension",
                &stern_steering_angle, &stern_wheel_speed)) {
        return;
    }

    port->getPosition(&port_steering_angle, NULL, NULL, NULL);
    if(0>computePod(body_vel, body_omega, body_pt, "port_suspension",
                &port_steering_angle, &port_wheel_speed)) {
        return;
    }

    starboard->getPosition(&starboard_steering_angle, NULL, NULL, NULL);
    if(0>computePod(body_vel, body_omega, body_pt, "starboard_suspension",
                &starboard_steering_angle, &starboard_wheel_speed)){
        return;
    }

    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    port->drive(port_steering_angle, port_wheel_speed);
    starboard->drive(starboard_steering_angle, -starboard_wheel_speed);
    stern->drive(stern_steering_angle, -stern_wheel_speed);
}

int Motion::computePod(Eigen::Vector2d body_vel, double body_omega, Eigen::Vector2d body_pt,
        const char *joint_name, double *steering, double *speed)
{
    tf::StampedTransform pod_tf;
    ros::Time current(0);
    try {
        listener.lookupTransform(child_frame_id, joint_name, current, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", joint_name, ex.what());
        return -1;
    }
    //ROS_DEBUG("%s at (%f, %f)", joint_name, port_tf.getOrigin().x(), port_tf.getOrigin().y());
    Eigen::Vector2d pod_pos( pod_tf.getOrigin().x(), pod_tf.getOrigin().y());
    Eigen::Vector2d pod_vect = pod_pos - body_pt;
    Eigen::Vector2d pod_perp(-pod_vect(1), pod_vect(0));
    Eigen::Vector2d pod_vel = body_vel + pod_perp*body_omega;

    *speed = pod_vel.norm();
    if(*speed>min_wheel_speed) {
        *steering = atan2(pod_vel(1), pod_vel(0));
        *speed /= M_PI*wheel_diameter;
        return 0;
    } else {
        *speed = 0;
        return 1;
    }

}



void Motion::doHomePods(void)
{
    home_pods_count = 3;
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    home_pods_action_server.acceptNewGoal();
    if(pods_enabled == true && command_source == COMMAND_SOURCE_NONE)
    {
        ROS_INFO("Home pods goal accepted");
        CANOpen::DS301CallbackObject
            hcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                    static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::homePodsComplete));
        port->home(hcb);
        starboard->home(hcb);
        stern->home(hcb);
    }
    else
    {
        const char * reason=pods_enabled ? "Command source not None" : "Pods not enabled";
        ROS_INFO("Home pods goal aborted - %s", reason);
        home_pods_result.homed.clear();
        home_pods_result.homed.push_back(false);
        home_pods_result.homed.push_back(false);
        home_pods_result.homed.push_back(false);
        home_pods_action_server.setAborted(home_pods_result, reason);
    }
}

void Motion::doHomeCarousel(void)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    home_carousel_action_server.acceptNewGoal();
    if(carousel_enabled)
    {
        ROS_INFO("Home carousel goal accepted");
        CANOpen::DS301CallbackObject
            hcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::homeCarouselComplete));
        carousel->home(hcb);
    }
    else
    {
        ROS_INFO("Home carousel goal aborted - not enabled");
        home_carousel_result.homed.clear();
        home_carousel_result.homed.push_back(false);
        home_carousel_action_server.setAborted(home_carousel_result, "Carousel not enabled");
    }
}

void Motion::homePodsComplete(CANOpen::DS301 &node)
{
    if(home_pods_count>0) home_pods_count--;
    ROS_INFO("homed servo %ld, count %d", node.node_id, home_pods_count);
    if(home_pods_action_server.isActive()) {
        home_pods_feedback.home_count = home_pods_count;
        home_pods_action_server.publishFeedback(home_pods_feedback);
        if(home_pods_count == 0) {
            home_pods_action_server.setSucceeded(home_pods_result);
            port->drive(0, 0);
            starboard->drive(0, 0);
            stern->drive(0, 0);
        }
    }
}

void Motion::homeCarouselComplete(CANOpen::DS301 &node)
{
    ROS_INFO("homed carousel");
    home_carousel_action_server.setSucceeded(home_carousel_result);  
}

bool Motion::ready(void)
{
    return port->ready() && starboard->ready() && stern->ready() && carousel->ready();
}

bool Motion::enableWheelPodsCallback(platform_motion_msgs::Enable::Request &req,
                                     platform_motion_msgs::Enable::Response &resp)
{
    bool notified=false;
    if(req.state==pods_enabled){
        ROS_DEBUG("Pods already in state %s", pods_enabled?"enabled":"disabled");
    }
    boost::unique_lock<boost::mutex> lock(enable_pods_mutex);
    boost::system_time now=boost::get_system_time();
    if(home_pods_action_server.isActive() && req.state == false){
        home_pods_action_server.setAborted(home_pods_result, "Paused");
    }
    enable_pods(req.state);
    notified = enable_pods_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    if(!notified)
    {
        if( (port->enabled() == req.state) &&
            (starboard->enabled() == req.state) &&
            (stern->enabled() == req.state) )
        {
            pods_enabled = req.state;
            // not really, but successful anyway
            notified=true;
        }
        else
        {
            ROS_ERROR("Wheel pods enable unsuccessful time out");
        }
    }
    resp.state = pods_enabled;
    return notified;
}

bool Motion::enableCarouselCallback(platform_motion_msgs::Enable::Request &req,
                                    platform_motion_msgs::Enable::Response &resp)
{
    bool notified=false;
    if(req.state==carousel_enabled){
        ROS_DEBUG("Carousel already in state %s", carousel_enabled?"enabled":"disabled");
    }
    boost::unique_lock<boost::mutex> lock(enable_carousel_mutex);
    boost::system_time now=boost::get_system_time();
    if(home_carousel_action_server.isActive() && req.state == false){
        home_carousel_action_server.setAborted(home_carousel_result, "Paused");
    }
    enable_carousel(req.state);
    notified = enable_carousel_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    if(!notified)
    {
        if( carousel->enabled() == req.state )
        {
            carousel_enabled=req.state;
            // not really, but successful anyway
            notified=true;
        }
        else
        {
            ROS_ERROR("Carousel enable unsuccessful time out");
        }
    }
    resp.state = carousel_enabled;
    return notified;
}


void Motion::podEnableStateChange(CANOpen::DS301 &node)
{
    if(enable_pods_count>0) --enable_pods_count;
    ROS_INFO("Enable report from %ld, count=%d", node.node_id, enable_pods_count);
    if(enable_pods_count == 0) {
        boost::unique_lock<boost::mutex> lock(enable_pods_mutex);
        pods_enabled = desired_pod_state;
        enable_pods_cond.notify_all();
    }
}

void Motion::carouselEnableStateChange(CANOpen::DS301 &node)
{
    ROS_INFO("Enable report from %ld, carousel", node.node_id);
    boost::unique_lock<boost::mutex> lock(enable_carousel_mutex);
    carousel_enabled = desired_carousel_state;
    enable_carousel_cond.notify_all();
}

void Motion::enable_pods(bool state)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    ROS_INFO( "enable pods: %s",  state?"true":"false");
    enable_pods_count=6;
    desired_pod_state = state;

    CANOpen::DS301CallbackObject
        ecb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::podEnableStateChange));
    port->enable(state, ecb);
    starboard->enable(state, ecb);
    stern->enable(state, ecb);
}

void Motion::enable_carousel(bool state)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    ROS_INFO( "enable carousel: %s",  state?"true":"false");
    desired_carousel_state = state;

    CANOpen::DS301CallbackObject
        ecb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::carouselEnableStateChange));
    carousel->enable(state, ecb);
}

void Motion::pvCallback(CANOpen::DS301 &node)
{
    if(pv_counter>0 && --pv_counter==0) {
        ros::Time current_time = ros::Time::now();

        double port_steering, starboard_steering, stern_steering;
        double port_steering_velocity, starboard_steering_velocity,
               stern_steering_velocity;
        double port_wheel, starboard_wheel, stern_wheel;
        double port_wheel_velocity, starboard_wheel_velocity,
               stern_wheel_velocity;

        port->getPosition(&port_steering, &port_steering_velocity,
                &port_wheel, &port_wheel_velocity);
        starboard->getPosition(&starboard_steering, &starboard_steering_velocity,
                &starboard_wheel, &starboard_wheel_velocity);
        stern->getPosition(&stern_steering, &stern_steering_velocity,
                &stern_wheel, &stern_wheel_velocity);

        starboard_wheel *= -1;
        starboard_wheel_velocity *= -1;

        stern_wheel *= -1;
        stern_wheel_velocity *= -1;

        sensor_msgs::JointState joints;
        joints.header.frame_id="base_link";
        joints.header.stamp=current_time;
        joints.header.seq=joint_seq++;

        joints.name.push_back("port_steering_joint");
        joints.position.push_back(port_steering);
        joints.velocity.push_back(port_steering_velocity);

        joints.name.push_back("port_axle");
        joints.position.push_back(port_wheel*2*M_PI);
        joints.velocity.push_back(port_wheel_velocity*2*M_PI);

        joints.name.push_back("starboard_steering_joint");
        joints.position.push_back(starboard_steering);
        joints.velocity.push_back(starboard_steering_velocity);

        joints.name.push_back("starboard_axle");
        joints.position.push_back(starboard_wheel*2*M_PI);
        joints.velocity.push_back(starboard_wheel_velocity*2*M_PI);

        joints.name.push_back("stern_steering_joint");
        joints.position.push_back(stern_steering);
        joints.velocity.push_back(stern_steering_velocity);

        joints.name.push_back("stern_axle");
        joints.position.push_back(stern_wheel*2*M_PI);
        joints.velocity.push_back(stern_wheel_velocity*2*M_PI);

        joints.name.push_back("carousel_joint");
        joints.position.push_back((carousel->position*2*M_PI/carousel_encoder_counts)+carousel_offset);
        joints.velocity.push_back(carousel->velocity*2*M_PI/10./carousel_encoder_counts);

        joint_state_pub.publish(joints);

    }
}

void Motion::gpioCallback(CANOpen::CopleyServo &svo, uint16_t old_pins, uint16_t new_pins)
{
    platform_motion_msgs::GPIO gpio;

    gpio.servo_id = svo.node_id;
    gpio.previous_pin_states = old_pins;
    gpio.new_pin_states = new_pins;
    gpio.pin_mask = 0xFFFF;
    gpio_pub.publish(gpio);
}

void Motion::gpioSubscriptionCallback(const platform_motion_msgs::GPIO::ConstPtr gpio)
{
    if((unsigned long)gpio->servo_id == carousel->node_id) {
        boost::unique_lock<boost::mutex> lock(CAN_mutex);
        uint16_t current = carousel->getOutputPins();
        uint16_t set   = gpio->pin_mask & ( gpio->new_pin_states & ~current);
        uint16_t clear = gpio->pin_mask & (~gpio->new_pin_states &  current);
        carousel->output(set, clear);
    } else {
            ROS_ERROR("Unknown servo_id in GPIO: %d", gpio->servo_id);
            return;
    }
}

void Motion::syncCallback(CANOpen::SYNC &sync)
{
    // reset the pv_counter to wait for all the servos to report status
    pv_counter=7;

    // reset the more data sent flag so we can send more data when needed
    this->moreDataSent = false;

    // increment the debug print counter
    m_debugPrintCounter++;

    // check to see if we're ready to start a coordinated pvt move
    // assuming we're in a mode that uses pvt moves..
    if( command_source == COMMAND_SOURCE_PLANNER )
    {
        if(port->getNeedsToStart() &&
           starboard->getNeedsToStart() &&
           stern->getNeedsToStart())
        {
            // if all the wheel pods are ready to go, tell them to start
            // moving. this means they should all start at the same time!
            port->startMoving();
            starboard->startMoving();
            stern->startMoving();
        }
    }

    /*
    if(carousel_enabled)
        pv_counter += 1;
    if(pods_enabled)
        pv_counter += 6;
        */
    if(carousel->ready()) {
        if(!carousel_setup ) {
            /* Any carousel-specific setup could go here */
            carousel_setup=true;
        }
        if(!gpio_enabled) {
            carousel->inputPinPullups(0x003f);
            std::vector<uint8_t> parameters;
            /* Calling any of these crahses the amp
            carousel->outputPinFunction(1, CANOpen::Manual, parameters, false);
            carousel->outputPinFunction(2, CANOpen::Manual, parameters, false);
            carousel->outputPinFunction(3, CANOpen::Manual, parameters, false);
            */
            gpio_enabled=true;
        }
    }
}

void Motion::reconfigureCallback(PlatformParametersConfig &config, uint32_t level)
{
    wheel_diameter = config.wheel_diameter;

    if( CAN_fd>0 ) {
        boost::unique_lock<boost::mutex> lock(CAN_mutex);
        port->setSteeringOffset(config.port_steering_offset);
        starboard->setSteeringOffset(config.starboard_steering_offset);
        stern->setSteeringOffset(config.stern_steering_offset);
        carousel_offset = config.carousel_offset;
        if(carousel->ready()) {
            carousel->setPosition(
                    (desired_carousel_position - carousel_offset)*
                    carousel_encoder_counts/2./M_PI);
        }
    }
}

void Motion::carouselCallback(const std_msgs::Float64::ConstPtr fmsg)
{
    desired_carousel_position = fmsg->data; 
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    carousel->setPosition(
            (desired_carousel_position - carousel_offset)*
            carousel_encoder_counts/2./M_PI);
}

void Motion::statusPublishCallback(const ros::TimerEvent& event)
{
    if(carousel_setup) {
        platform_motion_msgs::GPIO gpio;
        gpio.servo_id = carousel->node_id;
        gpio.previous_pin_states = carousel->getInputPins();
        gpio.new_pin_states = gpio.previous_pin_states;
        gpio.pin_mask = 0xFFFF;
        gpio_pub.publish(gpio);

        std_msgs::Float64 voltage;
        voltage.data = carousel->bus_voltage;
        battery_voltage_pub.publish(voltage);
    }

    platform_motion_msgs::ServoStatus status;
    std::stringstream status_stream;
    std::stringstream mode_stream;

    port->steering.writeStatus(status_stream);
    port->steering.writeMode(mode_stream);
    status.servo_id=port->steering.node_id;
    status.status=status_stream.str();
    status.mode=mode_stream.str();
    status_pub.publish(status);
    status_stream.str("");
    mode_stream.str("");

    port->wheel.writeStatus(status_stream);
    port->wheel.writeMode(mode_stream);
    status.servo_id=port->wheel.node_id;
    status.status=status_stream.str();
    status.mode=mode_stream.str();
    status_pub.publish(status);
    status_stream.str("");
    mode_stream.str("");

    starboard->steering.writeStatus(status_stream);
    starboard->steering.writeMode(mode_stream);
    status.servo_id=starboard->steering.node_id;
    status.status=status_stream.str();
    status.mode=mode_stream.str();
    status_pub.publish(status);
    status_stream.str("");
    mode_stream.str("");

    starboard->wheel.writeStatus(status_stream);
    starboard->wheel.writeMode(mode_stream);
    status.servo_id=starboard->wheel.node_id;
    status.status=status_stream.str();
    status.mode=mode_stream.str();
    status_pub.publish(status);
    status_stream.str("");
    mode_stream.str("");

    stern->steering.writeStatus(status_stream);
    stern->steering.writeMode(mode_stream);
    status.servo_id=stern->steering.node_id;
    status.status=status_stream.str();
    status.mode=mode_stream.str();
    status_pub.publish(status);
    status_stream.str("");
    mode_stream.str("");

    stern->wheel.writeStatus(status_stream);
    stern->wheel.writeMode(mode_stream);
    status.servo_id=stern->wheel.node_id;
    status.status=status_stream.str();
    status.mode=mode_stream.str();
    status_pub.publish(status);
    status_stream.str("");
    mode_stream.str("");

    carousel->writeStatus(status_stream);
    carousel->writeMode(mode_stream);
    status.servo_id = carousel->node_id;
    status.status = status_stream.str();
    status.mode = mode_stream.str();
    status_pub.publish(status);
}

void Motion::statusCallback(CANOpen::DS301 &node)
{
    CANOpen::CopleyServo *svo = static_cast<CANOpen::CopleyServo*>(&node);
    platform_motion_msgs::ServoStatus status;
    std::stringstream status_stream;
    std::stringstream mode_stream;

    svo->writeStatus(status_stream);
    svo->writeMode(mode_stream);
    status.servo_id=svo->node_id;
    status.status = status_stream.str();
    status.mode = mode_stream.str();
    status_pub.publish(status);
}

void Motion::plannedPathCallback(const platform_motion_msgs::Path::ConstPtr path)
{
    if(command_source != COMMAND_SOURCE_PLANNER)
    {
        // if the planner hasn't been put in charge, don't listen to it
        return;
    }
    // take the new path and insert it into the plannedPath
    // list in the right spot.
    // first, remove everything in the list that is now made obsolete
    // by the new planned path.
    ros::Time startTime = path->header.stamp;
    plannedPath.remove_if(
            [startTime](platform_motion_msgs::Knot p)
            {
                return p.header.stamp >= startTime;
            }
    );

    // now copy all the path segments to the end of the list.
    plannedPath.insert(plannedPath.end(), path->knots.begin(), path->knots.end());

    // add a "safety" segment to the very end that will stop the robot
    // one second after the end of the path. this seems like a good thing
    platform_motion_msgs::Knot lastPose = plannedPath.back();
    lastPose.header.stamp += ros::Duration(1.0);
    lastPose.twist.linear.x = 0;
    lastPose.twist.linear.y = 0;
    lastPose.twist.linear.z = 0;
    lastPose.twist.angular.x = 0;
    lastPose.twist.angular.y = 0;
    lastPose.twist.angular.z = 0;
    plannedPath.push_back(lastPose);

    // check to see if the wheel pods are out of data. if they are,
    // send a couple of segments to get things moving.
    if((port->getMinBufferDepth() < 3) ||
            (starboard->getMinBufferDepth() < 3) ||
            (stern->getMinBufferDepth() < 3))
    {
        // just send one to get the ball rolling. they should ask for more
        // right away (I think?)
        sendPvtSegment();
    }

}

void Motion::moreDataNeededCallback(CANOpen::DS301 &node)
{
    // only send more data once per sync pulse. this way we send data to
    // all the servos as soon as one says it needs more
    if(!this->moreDataSent)
    {
        // set the flag so we don't send data for every callback call this
        // sync fame
        this->moreDataSent = true;

        // send the next segment
        sendPvtSegment();
    }
}

void Motion::errorCallback(CANOpen::DS301 &node)
{
    // handle a pvt related error here!
    // for now, just print out the error...
    ROS_ERROR("Servo with id %lu had the following error:%s",
            node.node_id,
            static_cast<CANOpen::CopleyServo*>(&node)->getLastError().c_str());
}

std::list<Motion::BodySegment> Motion::pathToBody(std::list<platform_motion_msgs::Knot> &path)
{
    std::list<BodySegment> retval;

    // store the current steering angles as a way to initialize last valid steering
    // angle
    double portLastValidAngle, starboardLastValidAngle, sternLastValidAngle;
    port->getPosition(&portLastValidAngle, nullptr, nullptr, nullptr);
    starboard->getPosition(&starboardLastValidAngle, nullptr, nullptr, nullptr);
    stern->getPosition(&sternLastValidAngle, nullptr, nullptr, nullptr);

    // look up the transforms
    tf::StampedTransform pod_tf;
    // asking for time 0 asks for the most recent transform
    ros::Time zero(0);
    try {
        listener.lookupTransform(child_frame_id, "port_suspension", zero, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", "port_suspension", ex.what());
        return retval;
    }
    Eigen::Vector3d port_pos;
    tf::vectorTFToEigen( pod_tf.getOrigin(), port_pos);
    try {
        listener.lookupTransform(child_frame_id, "starboard_suspension", zero, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", "starboard_suspension", ex.what());
        return retval;
    }
    Eigen::Vector3d starboard_pos;
    tf::vectorTFToEigen( pod_tf.getOrigin(), starboard_pos);
    try {
        listener.lookupTransform(child_frame_id, "stern_suspension", zero, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", "stern_suspension", ex.what());
        return retval;
    }
    Eigen::Vector3d stern_pos;
    tf::vectorTFToEigen( pod_tf.getOrigin(), stern_pos);

    // make sure that the last thing in the path is a segment with zero velocity
    platform_motion_msgs::Knot last = path.back();
    last.twist.linear.x = 0;
    last.twist.linear.y = 0;
    last.twist.angular.z = 0;
    last.header.stamp += ros::Duration(0.25);
    path.push_back(last);


    // iterate over the path. this is slightly tricky because we want two consecutive
    // elements at a time
    platform_motion_msgs::Knot previous;
    previous.twist.linear.x = 0;
    previous.twist.linear.y = 0;
    previous.twist.angular.z = 0;
    previous.header.stamp = path.begin()->header.stamp-ros::Duration(0.250);

    // debug counter!
    int j = 0;

    for(auto current = path.begin(), next = ++path.begin();
            next != path.end();
            current++, next++)
    {
        BodySegment segment;
        segment.time = (*current).header.stamp;
        segment.port = pathToPod(previous, *current, *next, port_pos, portLastValidAngle);
        segment.starboard = pathToPod(previous, *current, *next, starboard_pos, starboardLastValidAngle);
        segment.stern = pathToPod(previous, *current, *next, stern_pos, sternLastValidAngle);

        retval.push_back(segment);
        previous = *current;

        ROS_DEBUG("%d: stern:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", j, segment.stern.steeringAngle, segment.stern.steeringVelocity, segment.stern.wheelDistance, segment.stern.wheelVelocity, segment.stern.duration);
        j++;
    }

    return retval;
}

// helper function needed only for pathToPod below
double unwrap(double theta)
{
    if(theta < -M_PI)
        theta += 2*M_PI;
    if(theta > M_PI)
        theta -= 2*M_PI;
    return theta;
}

Eigen::Vector2d Motion::podVelocity(const platform_motion_msgs::Knot &current, Eigen::Vector3d pod_pos)
{
    Eigen::Quaterniond rot;
    tf::quaternionMsgToEigen( current.pose.orientation, rot );

    Eigen::Matrix3d omegacross;
    omegacross << 0, -current.twist.angular.z, 0,
                  current.twist.angular.z, 0, 0,
                  0, 0, 1;

    Eigen::Vector3d Vbody;
    tf::vectorMsgToEigen( current.twist.linear, Vbody );

    return (omegacross * (rot * pod_pos) + Vbody).head(2);
}

PodSegment Motion::pathToPod(platform_motion_msgs::Knot &previous, platform_motion_msgs::Knot &current, platform_motion_msgs::Knot &next, Eigen::Vector3d pod_pos, double &lastValidSteeringAngle)
{
    // this is a helper that does the actual math for each pod segment.
    // this math comes from contradict's path2local.py and I'm using those vairable names
    // for the most part. sorry about that.
    // compute the differentials
    double dx = current.pose.position.x - previous.pose.position.x;
    double dy = current.pose.position.y - previous.pose.position.y;
    Eigen::Quaterniond q;
    double previousTheta = tf::getYaw( previous.pose.orientation );
    double currentTheta = tf::getYaw( current.pose.orientation );
    double nextTheta = tf::getYaw( next.pose.orientation );
    double dtheta = unwrap(currentTheta - previousTheta);
    double dt_prev = (current.header.stamp - previous.header.stamp).toSec();
    double dt_next = (next.header.stamp - current.header.stamp).toSec();

    PodSegment retval;
    retval.duration = dt_next;

    Eigen::Vector2d Vpod = podVelocity(current, pod_pos);

    double ksidot = Vpod.norm();

    if(ksidot < 0.001)
    {
        // if ksidot is nearly zero, steering angle and velocity will be wrong!
        // set steering velocity to 0 and steering angle to the last valid and
        // return early. that's all we can do for now.
        retval.steeringAngle = lastValidSteeringAngle;
        retval.steeringVelocity = 0.0;
        retval.wheelDistance = 0.0;
        retval.wheelVelocity = 0.0;
        return retval;
    }

    double phi = unwrap(atan2(Vpod[1], Vpod[0]) - currentTheta);

    // add motion due to rotation about body axis of pod
    Eigen::Matrix2d rot;
    rot << cos(currentTheta), -sin(currentTheta),
           sin(currentTheta), cos(currentTheta);

    Eigen::Matrix2d drot;
    drot << cos(dtheta)-1, -sin(dtheta),
            sin(dtheta), cos(dtheta)-1;

    Eigen::Vector2d rdx = drot * rot * pod_pos.head(2);

    dx += rdx(0);
    dy += rdx(1);
    // we determined .001 by looking at the limit. it might want to be a parameter or
    // a constant...
    double dksi;
    if(fabs(dtheta) < 0.001)
    {
        dksi = sqrt(pow(dx, 2) + pow(dy, 2));
    }
    else
    {
        dksi = fabs(dtheta) * sqrt(pow(dx, 2) + pow(dy, 2)) / sqrt(2 * (1.0 - cos(dtheta)));
    }

    double alpha = unwrap(phi + currentTheta);

    Eigen::Vector2d Vpod_prev = podVelocity(previous, pod_pos);
    Eigen::Vector2d Vpod_next = podVelocity(next, pod_pos);
    Eigen::Vector2d Vdot = ((Vpod-Vpod_prev)/dt_prev + (Vpod_next-Vpod)/dt_next)/2.;

    double phidot = (Vdot[0] * cos(-alpha - M_PI/2.0) - Vdot[1] * sin(-alpha - M_PI/2.0)) /
        ksidot - current.twist.angular.z;

    // we've actually computed everything now! stick the results into retval!
    retval.steeringAngle = phi;
    retval.steeringVelocity = phidot;
    retval.wheelDistance = dksi;
    retval.wheelVelocity = ksidot;

    // update last valid steering angle in case we hit another zero velocity path.
    lastValidSteeringAngle = phi;

    return retval;
}

// function definition interlude!
void cubicInterpolate(double x0, double xDot0, double x1, double xDot1, double alpha, double *x, double *v)
{
    // this comes from something we worked out on the board. it could be
    // a little easier to read..
    double c = -6.0 * (x1 - 0.5 * xDot1 - x0 - 0.5 * xDot0);
    double b = xDot1 - xDot0 - c;

    *x = x0 + xDot0 * alpha + 0.5 * b * pow(alpha, 2.0) + (1.0/3.0) * c * pow(alpha, 3.0);
    *v = xDot0 + b * alpha +  c * pow(alpha, 2.0);
}

PodSegment interpolatePodSegment(const PodSegment &first, const PodSegment &second, const PodSegment &last, double dadt, double alphaNow, double alphaLast)
{
    PodSegment retval;
    cubicInterpolate(
            first.steeringAngle, first.steeringVelocity / dadt,
            second.steeringAngle, second.steeringVelocity / dadt,
            alphaNow,
            &retval.steeringAngle,
            &retval.steeringVelocity
    );
    retval.steeringVelocity *= dadt;

    cubicInterpolate(
            0, first.wheelVelocity / dadt,
            second.wheelDistance, second.wheelVelocity / dadt,
            alphaNow,
            &retval.wheelDistance,
            &retval.wheelVelocity
            );
    double lastWheelDistance, lastWheelVelocity;
    cubicInterpolate(
            0, first.wheelVelocity / dadt,
            second.wheelDistance, second.wheelVelocity / dadt,
            alphaLast,
            &lastWheelDistance,
            &lastWheelVelocity
            );
    retval.wheelDistance -= lastWheelDistance;
    retval.wheelVelocity *= dadt;
    return retval;
}

Motion::BodySegment Motion::interpolatePodSegments(const BodySegment &first, const BodySegment &second, const BodySegment &last, ros::Time now)
{
    double dadt=1./(second.time-first.time).toSec();
    double alphaNow = (now - first.time).toSec()*dadt;
    double alphaLast = (last.time - first.time).toSec()*dadt;

    BodySegment retval;
    retval.port = interpolatePodSegment( first.port, second.port, last.port, dadt, alphaNow, alphaLast);
    retval.starboard = interpolatePodSegment( first.starboard, second.starboard, last.starboard, dadt, alphaNow, alphaLast);
    retval.stern = interpolatePodSegment( first.stern, second.stern, last.stern, dadt, alphaNow, alphaLast);

    return retval;
}

void Motion::sendPvtSegment()
{
    // get the steering angles (and a bunch of other crud) so we can
    // tell the steering not to move for each pod, if needed.
    // this seems like a good thing.
    double port_steering, starboard_steering, stern_steering;
    double port_steering_velocity, starboard_steering_velocity,
           stern_steering_velocity;
    double port_wheel, starboard_wheel, stern_wheel;
    double port_wheel_velocity, starboard_wheel_velocity,
           stern_wheel_velocity;

    port->getPosition(&port_steering, &port_steering_velocity,
            &port_wheel, &port_wheel_velocity);
    starboard->getPosition(&starboard_steering, &starboard_steering_velocity,
            &starboard_wheel, &starboard_wheel_velocity);
    stern->getPosition(&stern_steering, &stern_steering_velocity,
            &stern_wheel, &stern_wheel_velocity);

    // for now, only the planner should be sending pvt segments.
    // if the planner isn't in charge, don't send segments, including
    // the stop segments. this will keep the zero velocity segments from
    // stomping all over other commands from other sources.
    // if the joystick and visual servo ever use pvt mode, this will
    // have to change.
    if( command_source == COMMAND_SOURCE_PLANNER )
    {
        BodySegment newSegment;

        // make sure the planned path has 2 or more segments in it. if it only has one
        // segment, just clear it.
        if(runningPath.size() == 1)
        {
            runningPath.clear();
        }

        // send the next pvt segment to all the wheelpods.
        if(runningPath.size() > 0)
        {
            // if we've got some path to follow, follow it.
            // get the next two path segments out of the path
            BodySegment first = runningPath.front();
            BodySegment second = *(++runningPath.begin());
            ros::Time now=lastSegmentSent.time+ros::Duration(lastSegmentSent.port.duration);

            ROS_DEBUG_STREAM("first: " << first.time << " second: " << second.time << " now: " << now << " new: " << newSegment.time);

            if(now < first.time)
            {
                ROS_DEBUG("last segment too old, sending first blindly");

                // if the last segment we sent was before this path, just send
                // the first segment. that's all we can really do.
                newSegment = first;

            }
            else if(now>second.time)
            {
                ROS_ERROR("past second");
                return;
            }
            else
            {
                if(lastSegmentSent.time == first.time && now == second.time) {
                    ROS_DEBUG("No interpolation needed");
                    newSegment = second;
                    runningPath.pop_front();
                } else {
                    // compute the distance with the fancy cubic interpolation thing.
                    ROS_DEBUG("interpolating to now");
                    newSegment = interpolatePodSegments(first, second, lastSegmentSent, now);
                }
                if((second.time - now).toSec() < 0.255)
                {
                    ROS_DEBUG("interpolated close enough, duration to second");

                    double duration = (second.time - now).toSec();

                    newSegment.port.duration = duration;
                    newSegment.starboard.duration = duration;
                    newSegment.stern.duration = duration;

                    // we're also past the first segment, so remove it from the
                    // path.
                    runningPath.pop_front();
                }
                else
                {
                    ROS_DEBUG("computing partial duration");

                    // send a fixed time step
                    // this prevents us from having a strange tiny duration at the end
                    double numSteps = ceil((second.time - first.time).toSec() / 0.255);
                    double step = (second.time - first.time).toSec() / numSteps;

                    // the next segment is step away from the last one in time.
                    newSegment.port.duration = step;
                    newSegment.starboard.duration = step;
                    newSegment.stern.duration = step;

                }
            }

            newSegment.time = now;

        // some debug printing. probably shouldn't check this in
        }
        else
        {
            ROS_DEBUG("out of path segments, sending zeros");

            // if there is no path to follow, keep the buffers full of zeros
            // this means we're constantly actually filling the buffer and
            // don't have to worry about it being empty. that seems like
            // a good thing.
            // we'll just give all the wheel pods the same segment.

            PodSegment segment;
            segment.steeringAngle = 0.0;
            segment.steeringVelocity = 0.0;
            segment.wheelDistance = 0.0;
            segment.wheelVelocity = 0.0;
            segment.duration = .25;

            // reset the steering angle for each pod so we don't move the
            // steering.
            segment.steeringAngle = port_steering;
            newSegment.port = segment;
            segment.steeringAngle = starboard_steering;
            newSegment.starboard = segment;
            segment.steeringAngle = stern_steering;
            newSegment.stern = segment;
            newSegment.time = lastSegmentSent.time + ros::Duration(lastSegmentSent.port.duration);
        }

        // make sure the durations are all less than or equal to .255!!
        double minDuration =
            std::min( 0.250,
                 std::min(newSegment.port.duration,
                     std::min(newSegment.starboard.duration, newSegment.stern.duration)));

        double maxDuration =
                 std::max(newSegment.port.duration,
                     std::max(newSegment.starboard.duration, newSegment.stern.duration));
        if( maxDuration > 0.255 )
        {
            ROS_DEBUG("Clipping duration");
            newSegment.port.duration = minDuration;
            newSegment.starboard.duration = minDuration;
            newSegment.stern.duration = minDuration;
        }

        ROS_DEBUG_STREAM("newSegment.time: " << newSegment.time );
        ROS_DEBUG("port:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", newSegment.port.steeringAngle, newSegment.port.steeringVelocity, newSegment.port.wheelDistance, newSegment.port.wheelVelocity, newSegment.port.duration);
        ROS_DEBUG("starboard:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", newSegment.starboard.steeringAngle, newSegment.starboard.steeringVelocity, newSegment.starboard.wheelDistance, newSegment.starboard.wheelVelocity, newSegment.starboard.duration);
        ROS_DEBUG("stern:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", newSegment.stern.steeringAngle, newSegment.stern.steeringVelocity, newSegment.stern.wheelDistance, newSegment.stern.wheelVelocity, newSegment.stern.duration);

        // now just send the new segment, whatever it was.
        port->move(newSegment.port);
        starboard->move(newSegment.starboard);
        stern->move(newSegment.stern);

        lastSegmentSent = newSegment;
    }
}

}
