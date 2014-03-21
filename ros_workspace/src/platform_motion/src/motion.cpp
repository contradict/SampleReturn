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
    scaryTestModeEnabled(false),
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

    scary_test_mode_sub = nh_.subscribe(
            "scary_test_mode",
            2,
            &Motion::scaryTestModeCallback,
            this
    );


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
    else if(req.source == std::string("ScaryTestMode"))
    {
        command_source = COMMAND_SOURCE_SCARY_TEST_MODE;
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
            case COMMAND_SOURCE_SCARY_TEST_MODE:
                resp.source = "ScaryTestMode";
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
    if((command_source == COMMAND_SOURCE_SCARY_TEST_MODE) ||
        (command_source == COMMAND_SOURCE_PLANNER))
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

void Motion::plannedPathCallback(const nav_msgs::Path::ConstPtr path)
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
    /*
     * NOTE: This is currently commented out because it isn't right!
     * plannedPath used to be a list of PosedStamped, but it isn't
     * any longer because PosedStamped doesn't contain sufficient information
     * to be useful. Once the data type of the path message is changed, it
     * should be pretty easy to figure out how to translate the contents of
     * that message into PathSegments. in the mean time, I'm developing the
     * scaryTestModeCallback below, so refer to it for the latest code that
     * is actually likely to work...
    ros::Time startTime = path->poses[0].header.stamp;
    plannedPath.remove_if(
            [startTime](geometry_msgs::PoseStamped p)
            {
                return p.header.stamp >= startTime;
            }
    );

    // now copy all the path segments to the end of the list.
    for(auto pose : path->poses)
    {
        plannedPath.push_back(pose);
    }

    // add a "safety" segment to the very end that will stop the robot
    // one second after the end of the path. this seems like a good thing
    geometry_msgs::PoseStamped lastPose = plannedPath.back();
    lastPose.header.stamp += ros::Duration(1.0);
    plannedPath.push_back(lastPose);

    // check to see if the wheel pods are out of data. if they are,
    // send a couple of segments to get things moving.
    if((port->getMinBufferDepth() < 3) ||
            (starboard->getMinBufferDepth() < 3) ||
            (stern->getMinBufferDepth() < 3))
    {
        // just send one to get the ball rolling. they should ask for more
        // right away (I think?)
        sendPvtSegment(ros::Time::now());
    }
    */
}

void Motion::scaryTestModeCallback(const std_msgs::Bool::ConstPtr enable)
{
    if(command_source != COMMAND_SOURCE_SCARY_TEST_MODE)
    {
        return;
    }

    scaryTestModeEnabled = enable->data;

    // clear the planned path.
    plannedPath.clear();

    // keep track of when we started.
    scaryTestModeStartTime = ros::Time::now();

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
    lastSegmentSent.time = ros::Time::now() - ros::Duration(0.25);

    if(scaryTestModeEnabled)
    {
        // if we're supposed to be going, make a new scary test mode path
        std::list<PathSegment> path = computeScaryPath(scaryTestModeStartTime, 250, 3.0, 0.1);
        // debugging stuff! don't check this in!
        //auto temp = pathToBody(path);

        plannedPath = pathToBody(path);
    }

    // send a pvt segment to kick things off
    sendPvtSegment();
    sendPvtSegment();
    sendPvtSegment();
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

std::list<Motion::PathSegment> Motion::computeScaryPath(ros::Time time, int numPoints, double amplitude, double omega)
{
    // compute the time step between each point
    ros::Duration timeStep = ros::Duration((2.0*M_PI/omega)/((double)numPoints));

    std::list<PathSegment> retval;

    // this is based on the math in path2local.py. sorry the contradict-style variable names
    // declare lambdas for the loop.
    auto f = [] (double x) {return 2.0 * atan(x);};
    auto fprime = [] (double x) {return 2.0/(1.0 + pow(x, 2));};
    auto g = [] (double x, double y) {return (sqrt(pow(x, 2) + pow(y, 2)) - x) / y;};
    auto gprime = [] (double x, double y, double xdot, double ydot)
        {return (pow((pow(x, 2) + pow(y, 2)), -0.5) * (x * xdot + y * ydot) - xdot) / y -
            (sqrt(pow(x, 2) + pow(y,2)) -x) / pow(y, 2) * ydot;};

    // compute the first point so that we have a nice ramp up to the start of the
    // figure 8. start by computing the first point of the figure 8.
    double xDot0 = 2.0 * amplitude * omega;
    double yDot0 = 2.0 * amplitude * omega;
    double theta0 = f(g(xDot0, yDot0));
    double acceleration = 0.1; // acceleration in meters per second. should be parameter
    double startDistance = 0.5 * (pow(xDot0, 2.0) + pow(yDot0, 2.0)) / acceleration;
    double startDuration = sqrt(pow(xDot0, 2.0) + pow(yDot0, 2.0)) / acceleration;
    PathSegment start;
    start.time = time;
    start.xDot = 0.0;
    start.yDot = 0.0;
    start.thetaDot = 0.0;
    start.x = startDistance * cos(theta0 + M_PI);
    start.y = startDistance * sin(theta0 + M_PI);
    start.theta = theta0;

    retval.push_back(start);

    // account for the start up ramp by adding some to the actual start time
    ros::Time actualStartTime = time + ros::Duration(startDuration);

    // build the figure 8 part of the path
    for(int i = 0; i < numPoints; i++)
    {
        double t = (timeStep * i).toSec();
        PathSegment segment;
        segment.time = actualStartTime + timeStep * i;
        segment.x = 2.0 * amplitude * sin(omega * t);
        segment.xDot = 2.0 * amplitude * omega * cos(omega * t);
        double xdDot = -2.0 * amplitude * omega * omega * sin(omega * t);
        segment.y = amplitude * sin(2.0 * omega * t);
        segment.yDot = 2.0 * amplitude * omega * cos(2.0 * omega * t);
        double ydDot = -4.0 * amplitude * omega * omega * sin(2.0 * omega * t);
        segment.theta = f(g(segment.xDot, segment.yDot));
        segment.thetaDot = fprime(g(segment.xDot, segment.yDot)) *
            gprime(segment.xDot, segment.yDot, xdDot, ydDot);

        retval.push_back(segment);
    }

    // now add a ramp down to zero at the end.
    PathSegment end;
    double endDistance = 0.5 * (pow(retval.back().xDot, 2.0) + pow(retval.back().yDot, 2.0))
        / acceleration;
    double endDuration = sqrt(pow(retval.back().xDot, 2.0) + pow(retval.back().yDot, 2.0)) /
        acceleration;

    end.time = retval.back().time + ros::Duration(endDuration);
    end.xDot = 0.0;
    end.yDot = 0.0;
    end.thetaDot = 0.0;
    end.x = endDistance * cos(retval.back().theta) + retval.back().x;
    end.y = endDistance * sin(retval.back().theta) + retval.back().y;
    end.theta = retval.back().theta;

    retval.push_back(end);

    // debugging. should not be pushed commented in!
    //for(auto i : retval)
    //{
    //    ROS_ERROR("time: %d, x: %d, y: %d, theta: %d, xDot: %d, yDot: %d, thetaDot: %d",
    //            i.time.toSec(), i.x, i.y, i.theta, i.xDot, i.yDot, i.thetaDot
    //    );
    //}

    return retval;

}

std::list<Motion::BodySegment> Motion::pathToBody(std::list<PathSegment> &path)
{
    // store the current steering angles as a way to initialize last valid steering
    // angle
    double portLastValidAngle, starboardLastValidAngle, sternLastValidAngle;
    port->getPosition(&portLastValidAngle, nullptr, nullptr, nullptr);
    starboard->getPosition(&starboardLastValidAngle, nullptr, nullptr, nullptr);
    stern->getPosition(&sternLastValidAngle, nullptr, nullptr, nullptr);

    // make sure that the last thing in the path is a segment with zero velocity
    PathSegment last = path.back();
    last.xDot = 0;
    last.yDot = 0;
    last.thetaDot = 0;
    last.time += ros::Duration(0.25);
    path.push_back(last);

    std::list<BodySegment> retval;

    // iterate over the path. this is slightly tricky because we want two consecutive
    // elements at a time
    PathSegment previous;
    previous.xDot = 0;
    previous.yDot = 0;
    previous.thetaDot = 0;
    previous.time = path.begin()->time-ros::Duration(0.250);

    // debug counter!
    int j = 0;

    for(auto current = path.begin(), next = ++path.begin();
            next != path.end();
            current++, next++)
    {
        BodySegment segment;
        segment.time = (*current).time;
        segment.port = pathToPod(previous, *current, *next, "port_suspension", portLastValidAngle);
        segment.starboard = pathToPod(previous, *current, *next, "starboard_suspension", starboardLastValidAngle);
        segment.stern = pathToPod(previous, *current, *next, "stern_suspension", sternLastValidAngle);

        retval.push_back(segment);
        previous = *current;

        ROS_ERROR("%d: stern:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", j, segment.stern.steeringAngle, segment.stern.steeringVelocity, segment.stern.wheelDistance, segment.stern.wheelVelocity, segment.stern.duration);
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

PodSegment Motion::pathToPod(PathSegment &previous, PathSegment &current, PathSegment &next, const char *jointName, double &lastValidSteeringAngle)
{
    // this is a helper that does the actual math for each pod segment.
    // this math comes from contradict's path2local.py and I'm using those vairable names
    // for the most part. sorry about that.
    // compute the differentials
    double dx = current.x - previous.x;
    double dy = current.y - previous.y;
    double dtheta = unwrap(current.theta - previous.theta);
    double dt_prev = (current.time - previous.time).toSec();
    double dt_next = (next.time - current.time).toSec();

    PodSegment retval;
    retval.duration = dt_next;

    // look up the transform
    tf::StampedTransform pod_tf;
    // asking for time 0 asks for the most recent transform
    ros::Time zero(0);
    try {
        listener.lookupTransform(child_frame_id, jointName, zero, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", jointName, ex.what());
        // set the steering angle to the last valid and zero out everything else
        // this is all we can do here...
        retval.steeringAngle = lastValidSteeringAngle;
        retval.steeringVelocity = 0.0;
        retval.wheelDistance = 0.0;
        retval.wheelVelocity = 0.0;
        return retval;
    }

    Eigen::Vector2d pod_pos( pod_tf.getOrigin().x(), pod_tf.getOrigin().y());

    // I am not really a fan of Eigen's matrix initializers
    Eigen::Matrix2d rot;
    rot << cos(current.theta), sin(current.theta),
           -sin(current.theta), cos(current.theta);

    Eigen::Matrix2d drot;
    drot << 1.0-cos(dtheta), sin(dtheta),
           -sin(dtheta), 1.0-cos(dtheta);

    Eigen::Vector2d rdx = drot * rot * pod_pos;

    dx += rdx(0);
    dy += rdx(1);

    double Vx = (-current.thetaDot * pod_pos(1)) + current.xDot;
    double Vy = (current.thetaDot * pod_pos(0)) + current.yDot;

    double ksidot = sqrt(pow(Vx, 2) + pow(Vy, 2));

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

    double phi = unwrap(atan2(Vy, Vx) - current.theta);

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

    double alpha = unwrap(phi + current.theta);

    double Vx_prev = (-previous.thetaDot * pod_pos(1)) + previous.xDot;
    double Vy_prev = (previous.thetaDot * pod_pos(0)) + previous.yDot;
    double Vx_next = (-next.thetaDot * pod_pos(1)) + next.xDot;
    double Vy_next = (next.thetaDot * pod_pos(0)) + next.yDot;
    double Vdotx = ((Vx-Vx_prev)/dt_prev + (Vx_next-Vx)/dt_next)/2.;
    double Vdoty = ((Vy-Vy_prev)/dt_prev + (Vx_next-Vx)/dt_next)/2.;

    double phidot = (Vdotx * cos(-alpha - M_PI/2.0) - Vdoty * sin(-alpha - M_PI/2.0)) /
        ksidot - current.thetaDot;

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
double cubicInterpolate(double x0, double xDot0, double x1, double xDot1, double alpha)
{
    // this comes from something we worked out on the board. it could be
    // a little easier to read..
    double c = -6.0 * (x1 - 0.5 * xDot1 - x0 - 0.5 * xDot0);
    double b = xDot1 - xDot0 - c;

    return x0 + xDot0 * alpha + 0.5 * b * pow(alpha, 2.0) + (1.0/3.0) * c * pow(alpha, 3.0);
}

PodSegment interpolatePodSegments(PodSegment &first, PodSegment &second, double interpolationAmount, double previousInterpolationAmount)
{
    // a helper for sendPvtSegment. this should probably be declared in the header...
    PodSegment retval;
    retval.steeringAngle = cubicInterpolate(
            first.steeringAngle, first.steeringVelocity,
            second.steeringAngle, second.steeringVelocity,
            interpolationAmount
    );
    retval.steeringVelocity = first.steeringVelocity +
        interpolationAmount*(second.steeringVelocity - first.steeringVelocity);

    retval.wheelDistance =
        cubicInterpolate(
            0, first.wheelVelocity,
            second.wheelDistance, second.wheelVelocity,
            interpolationAmount) -
        cubicInterpolate(
            0, first.wheelVelocity,
            second.wheelDistance, second.wheelVelocity,
            previousInterpolationAmount
        );

    retval.wheelVelocity = first.wheelVelocity +
        interpolationAmount*(second.wheelVelocity - first.wheelVelocity);
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
    if((command_source == COMMAND_SOURCE_PLANNER) ||
       (command_source == COMMAND_SOURCE_SCARY_TEST_MODE))
    {
        BodySegment newSegment;

        // make sure the planned path has 2 or more segments in it. if it only has one
        // segment, just clear it.
        if(plannedPath.size() == 1)
        {
            plannedPath.clear();
        }

        // send the next pvt segment to all the wheelpods.
        if(plannedPath.size() > 0)
        {
            // if we've got some path to follow, follow it.
            // get the next two path segments out of the path
            BodySegment first = plannedPath.front();
            BodySegment second = *(++plannedPath.begin());
            ros::Time now=lastSegmentSent.time+ros::Duration(lastSegmentSent.port.duration);
            newSegment.time = now;

            if(now < first.time)
            {
                ROS_ERROR("last segment too old, sending first blindly");

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
                double alphaLast = (lastSegmentSent.time - first.time).toSec()/(second.time-first.time).toSec();
                double alphaNow = (now - first.time).toSec()/(second.time-first.time).toSec();
                if(alphaLast == 0. && alphaNow == 1) {
                    newSegment = second;
                } else {
                    // compute the distance with the fancy cubic interpolation thing.
                    newSegment.port = interpolatePodSegments(
                            first.port,
                            second.port,
                            alphaNow,
                            alphaLast
                            );

                    newSegment.starboard = interpolatePodSegments(
                            first.starboard,
                            second.starboard,
                            alphaNow,
                            alphaLast
                            );

                    newSegment.stern = interpolatePodSegments(
                            first.stern,
                            second.stern,
                            alphaNow,
                            alphaLast
                            );
                }
                if((second.time - now).toSec() < 0.255)
                {
                    ROS_ERROR("interpolated close enough, duration to second");

                    double duration = (second.time - now).toSec();

                    newSegment.port.duration = duration;
                    newSegment.starboard.duration = duration;
                    newSegment.stern.duration = duration;

                    // we're also past the first segment, so remove it from the
                    // path.
                    plannedPath.pop_front();
                }
                else
                {
                    ROS_ERROR("computing partial duration");

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
        }
        else
        {
            ROS_ERROR("out of path segments, sending zeros");

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
            newSegment.time = lastSegmentSent.time + ros::Duration(0.250);
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
            ROS_ERROR("Clipping duration");
            newSegment.port.duration = minDuration;
            newSegment.starboard.duration = minDuration;
            newSegment.stern.duration = minDuration;
        }

        // some debug printing. probably shouldn't check this in
        ROS_ERROR("port:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", newSegment.port.steeringAngle, newSegment.port.steeringVelocity, newSegment.port.wheelDistance, newSegment.port.wheelVelocity, newSegment.port.duration);
        ROS_ERROR("starboard:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", newSegment.starboard.steeringAngle, newSegment.starboard.steeringVelocity, newSegment.starboard.wheelDistance, newSegment.starboard.wheelVelocity, newSegment.starboard.duration);
        ROS_ERROR("stern:\nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", newSegment.stern.steeringAngle, newSegment.stern.steeringVelocity, newSegment.stern.wheelDistance, newSegment.stern.wheelVelocity, newSegment.stern.duration);

        // now just send the new segment, whatever it was.
        port->move(newSegment.port);
        starboard->move(newSegment.starboard);
        stern->move(newSegment.stern);

        lastSegmentSent = newSegment;
    }
}

}
