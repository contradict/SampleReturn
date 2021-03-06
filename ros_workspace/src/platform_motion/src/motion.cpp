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
#include <platform_motion_msgs/SelectMotionMode.h>
#include <platform_motion/PlatformParametersConfig.h>
#include <platform_motion_msgs/GPIO.h>
#include <platform_motion_msgs/ServoStatus.h>
#include <platform_motion_msgs/Path.h>
#include <motion/wheelpod.h>

#include <motion/motion.h>

#define POLL_TIMEOUT_MS 10

namespace platform_motion {

Motion::Motion() :
    m_debugPrintCounter(0),
    param_nh("~"),
    home_pods_action_server(nh_, "home_wheel_pods", false),
    home_carousel_action_server(nh_, "home_carousel", false),
    motion_mode(platform_motion_msgs::SelectMotionMode::Request::MODE_DISABLE),
    saved_mode(platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE),
    CAN_fd(-1),
    gpio_enabled(false),
    carousel_setup(false),
    pods_enabled(false),
    desired_pod_state(false),
    carousel_enabled(false),
    desired_carousel_state(false),
    pv_counter(0),
    joint_seq(0),
    moreDataSent(false),
    restartPvt(false),
    newPathReady(false)
{
    lastSegmentSent_.port.steeringAngle = 0.0;
    lastSegmentSent_.port.steeringVelocity = 0.0;
    lastSegmentSent_.port.wheelDistance = 0.0;
    lastSegmentSent_.port.wheelVelocity = 0.0;
    lastSegmentSent_.port.duration = 0.25;
    lastSegmentSent_.starboard.steeringAngle = 0.0;
    lastSegmentSent_.starboard.steeringVelocity = 0.0;
    lastSegmentSent_.starboard.wheelDistance = 0.0;
    lastSegmentSent_.starboard.wheelVelocity = 0.0;
    lastSegmentSent_.starboard.duration = 0.25;
    lastSegmentSent_.stern.steeringAngle = 0.0;
    lastSegmentSent_.stern.steeringVelocity = 0.0;
    lastSegmentSent_.stern.wheelDistance = 0.0;
    lastSegmentSent_.stern.wheelVelocity = 0.0;
    lastSegmentSent_.stern.duration = 0.25;
    lastSegmentSent_.time = ros::Time(0);

    ROS_INFO("Started in namespace %s", nh_.getNamespace().c_str());

    ROS_INFO("param in namespace %s", param_nh.getNamespace().c_str());

    param_nh.param("wheel_diameter", wheel_diameter, 0.330);
    param_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

    param_nh.param("center_pt_x", center_pt_x, 0.0);
    param_nh.param("center_pt_y", center_pt_y, 0.0);

    param_nh.param("min_wheel_speed", min_wheel_speed, 0.0005);

    param_nh.param("enable_wait_timeout_ms", enable_wait_timeout, 500);

    param_nh.param("plan_to_zero_decel", planToZeroDecel_, 2.0);
    param_nh.param("plan_to_zero_period", planToZeroPeriod_, 0.050);

    param_nh.param("port_steering_lock", portSteeringLock_, M_PI/3.);
    param_nh.param("starboard_steering_lock", starboardSteeringLock_, -M_PI/3.);
    param_nh.param("stern_steering_lock", sternSteeringLock_, 0.0);

    param_nh.param("steering_tolerance", steeringTolerance_, 0.01);
    param_nh.param("steering_accel", steeringAccel_, M_PI);
    param_nh.param("maximum_steering_velocity", maxSteeringVelocity_, M_PI/4.);
    param_nh.param("maximum_wheel_acceleration", maxWheelAcceleration_, 2.0);

    body_pt << center_pt_x, center_pt_y;
    enable_carousel_server = nh_.advertiseService("enable_carousel",
            &Motion::enableCarouselCallback, this);

    gpio_server = nh_.advertiseService("gpio_service",
            &Motion::gpioServiceCallback, this);

    motion_mode_server = nh_.advertiseService("CAN_select_motion_mode",
            &Motion::selectMotionModeCallback, this);
    planner_sub = nh_.subscribe("planner_command", 2, &Motion::plannerTwistCallback, this);
    joystick_sub = nh_.subscribe("joystick_command", 2, &Motion::joystickTwistCallback, this);
    servo_sub = nh_.subscribe("servo_command", 2, &Motion::servoTwistCallback, this);

    carousel_sub = nh_.subscribe("carousel_angle", 2, &Motion::carouselCallback,
            this);

    gpio_sub = nh_.subscribe("gpio_write", 2, &Motion::gpioSubscriptionCallback,
            this);

    planned_path_sub = nh_.subscribe("planned_path", 1, &Motion::plannedPathCallback, this);

    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("platform_joint_state", 1);

    gpio_pub = nh_.advertise<platform_motion_msgs::GPIO>("gpio_read", 1);

    battery_voltage_pub = nh_.advertise<std_msgs::Float64>("battery_voltage", 1);

    status_pub = nh_.advertise<platform_motion_msgs::ServoStatus>("CAN_status_word", 10);

    stitchedPath_pub_ = nh_.advertise<platform_motion_msgs::Path>("stitched_path", 1);

    motionMode_pub_ = nh_.advertise<platform_motion_msgs::SelectMotionModeResponse>("current_motion_mode", 1, true);

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
    boost::unique_lock<boost::mutex> pod_lock(enable_pods_mutex);
    while( pods_enabled == true )
        enable_pods_cond.timed_wait(pod_lock, boost::posix_time::milliseconds(enable_wait_timeout));
    enable_carousel(false);
    boost::unique_lock<boost::mutex> carousel_lock(enable_carousel_mutex);
    while( carousel_enabled == true )
        enable_carousel_cond.timed_wait(carousel_lock, boost::posix_time::milliseconds(enable_wait_timeout));
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
    // asking for time 0 asks for the most recent transform
    ros::Time most_recent(0);
    // look up the transforms
    tf::StampedTransform pod_tf;
    try {
        listener.lookupTransform(child_frame_id, "port_suspension", most_recent, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", "port_suspension", ex.what());
    }
    tf::vectorTFToEigen( pod_tf.getOrigin(), port_pos_);
    try {
        listener.lookupTransform(child_frame_id, "starboard_suspension", most_recent, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", "starboard_suspension", ex.what());
    }
    tf::vectorTFToEigen( pod_tf.getOrigin(), starboard_pos_);
    try {
        listener.lookupTransform(child_frame_id, "stern_suspension", most_recent, pod_tf );
    } catch( tf::TransformException ex) {
        ROS_ERROR("Error looking up %s: %s", "stern_suspension", ex.what());
    }
    tf::vectorTFToEigen( pod_tf.getOrigin(), stern_pos_);
}

void Motion::plannerTwistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    if(motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST)
        handleTwist(twist);
}

void Motion::joystickTwistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    if(motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK)
        handleTwist(twist);
}

void Motion::servoTwistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    if(motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO)
        handleTwist(twist);
}

std::string motion_mode_string(int mode)
{
    switch(mode)
    {
        case platform_motion_msgs::SelectMotionMode::Request::MODE_QUERY:
            return "Query";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_HOME:
            return "Home";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK:
            return "Joystick";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST:
            return "Planner Twist";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO:
            return "Servo";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT:
            return "Planner PVT";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE:
            return "Pause";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_RESUME:
            return "Resume";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK:
            return "Lock";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_UNLOCK:
            return "Unlock";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE:
            return "Enable";
        case platform_motion_msgs::SelectMotionMode::Request::MODE_DISABLE:
            return "Disable";
        default:
            return "Unknown";
    }
}

bool Motion::selectMotionModeCallback(platform_motion_msgs::SelectMotionMode::Request &req,
                                      platform_motion_msgs::SelectMotionMode::Response &resp)
{
    ROS_INFO("Requested motion mode %s", motion_mode_string(req.mode).c_str());
    // Handle query right away
    if( req.mode == platform_motion_msgs::SelectMotionMode::Request::MODE_QUERY )
    {
        ROS_INFO("Query mode %s", motion_mode_string(motion_mode).c_str());
        resp.mode = motion_mode;
        return true;
    }
    // Check states which have special exit conditions
    switch( motion_mode )
    {
        case platform_motion_msgs::SelectMotionMode::Request::MODE_QUERY:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_HOME:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT:
            // make sure PVT buffers are all cleared.
            port->wheel.clearPvtBuffer();
            port->steering.clearPvtBuffer();
            starboard->wheel.clearPvtBuffer();
            starboard->steering.clearPvtBuffer();
            stern->wheel.clearPvtBuffer();
            stern->steering.clearPvtBuffer();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE:
            if( req.mode != platform_motion_msgs::SelectMotionMode::Request::MODE_RESUME &&
                req.mode != platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE &&
                req.mode != platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK &&
                req.mode != platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE
                )
            {
                ROS_ERROR("Attempted mode transition other than RESUME, ENABLE, PAUSE or LOCK from PAUSE: %s", motion_mode_string(req.mode).c_str());
                return false;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_RESUME:
            // no special action needed
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK:
            if(req.mode != platform_motion_msgs::SelectMotionMode::Request::MODE_UNLOCK)
            {
                ROS_ERROR("Attempted mode transition other than UNLOCK from LOCK: %s", motion_mode_string(req.mode).c_str());
                return false;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_UNLOCK:
            // no special action needed
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE:
            // no special action needed
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_DISABLE:
            if(req.mode != platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE)
            {
                ROS_ERROR("Attempted mode transition other than ENABLE from DISABLE: %s", motion_mode_string(req.mode).c_str());
                return false;
            }
            break;
        default:
            ROS_FATAL("In unknown mode, serious error: %d", motion_mode);
    }
    bool podsResult;
    // Enter new state if possible
    switch(req.mode)
    {
        case platform_motion_msgs::SelectMotionMode::Request::MODE_QUERY:
            // already handled
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_HOME:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO:
            // These states can be entered from anywhere you can get out of
            // and need to take no immediate action
            motion_mode = req.mode;
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT:
            motion_mode = req.mode;
            setLastSegmentToCurrent( );
            primePVT();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE:
            if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_HOME ||
                motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK ||
                motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST ||
                motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO ||
                motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT ||
                motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE)
            {
                // remember where you were for resume and
                saved_mode = motion_mode;
                // take care of slowing down
                motion_mode = req.mode;
                handlePause();
            }
            else if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE )
            {
                motion_mode = req.mode;
            }
            else
            {
                ROS_ERROR( "Cannot PAUSE from %s", motion_mode_string(motion_mode).c_str() );
                return false;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_RESUME:
            // Can only enter RESUME if we were in PAUSE
            if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE )
            {
                // move immediately to saved mode
                motion_mode = saved_mode;
            }
            else
            {
                ROS_ERROR( "Attempted RESUME from state other than PAUSE: %s", motion_mode_string( motion_mode ).c_str() );
                return false;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK:
            // Can only LOCK if already PAUSED
            if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE )
            {
                motion_mode = req.mode;
                handleLock();
            }
            else
            {
                ROS_ERROR( "Attempted LOCK from state other than PAUSE: %s", motion_mode_string( motion_mode ).c_str() );
                return false;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_UNLOCK:
            // Can only UNLOCK if already LOCKED
            if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK )
            {
                motion_mode=platform_motion_msgs::SelectMotionMode::Request::MODE_UNLOCK;
                handleUnlock();
            }
            else
            {
                ROS_ERROR( "Attempted UNLOCK from state other than LOCK: %s", motion_mode_string( motion_mode ).c_str() );
                return false;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE:
            // can ENABLE from anywhere
            if( !handleEnablePods(true, &podsResult) )
            {
                //if the servos do not enter ENABLE, bad news...
                return false;
            }
            if( podsResult )
            {
                motion_mode = req.mode;
            }
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_DISABLE:
            // can DISABLE from anywhere
            if( !handleEnablePods(false, &podsResult) )
                return false;
            if( !podsResult )
                motion_mode = req.mode;
            break;
        default:
            ROS_ERROR("Unrecognized motion mode %d", req.mode);
            return false;
    }
    ROS_INFO("Selected motion mode %s", motion_mode_string(motion_mode).c_str());
    resp.mode=motion_mode;
    motionMode_pub_.publish( resp );
    return true;
}

void Motion::cancelHome( void )
{
    if( home_pods_action_server.isActive() ){
        home_pods_action_server.setAborted(home_pods_result, "Paused");
    }
    port->cancelHome();
    starboard->cancelHome();
    stern->cancelHome();
}

void Motion::handlePause( void )
{
    switch(saved_mode)
    {
        case platform_motion_msgs::SelectMotionMode::Request::MODE_HOME:
            cancelHome();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO:
            planToZeroTwist();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT:
            primePVT();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE:
            // nothing to do, already here
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE:
            // nothing to do, already safe
            break;
        default:
            ROS_ERROR("Unexpected state on pause entry: %s", motion_mode_string(motion_mode).c_str());
            break;
     }
}

void Motion::planToZeroTwist( void )
{
    double port_steering, starboard_steering, stern_steering;
    double port_wheel_velocity, starboard_wheel_velocity,
           stern_wheel_velocity;

    port->getPosition(&port_steering, NULL, NULL, &port_wheel_velocity);
    starboard->getPosition(&starboard_steering, NULL, NULL, &starboard_wheel_velocity);
    stern->getPosition(&stern_steering, NULL, NULL, &stern_wheel_velocity);

    double max_velocity = std::max(fabs(port_wheel_velocity),
                                   std::max(fabs(starboard_wheel_velocity),
                                       fabs(stern_wheel_velocity)));
    double planToZeroTime = max_velocity/planToZeroDecel_;
    if(planToZeroTime>planToZeroPeriod_)
    {
        int count=ceil(planToZeroTime/planToZeroPeriod_);

        double port_decel = port_wheel_velocity/count;
        double starboard_decel = starboard_wheel_velocity/count;
        double stern_decel = stern_wheel_velocity/count;

        ros::Rate loop(1./planToZeroPeriod_);
        for(int i=0;i<count;i++)
        {
            port_wheel_velocity -= port_decel;
            starboard_wheel_velocity -= starboard_decel;
            stern_wheel_velocity -= stern_decel;

            port->drive( port_steering, port_wheel_velocity );
            starboard->drive( starboard_steering, starboard_wheel_velocity );
            stern->drive( stern_steering, stern_wheel_velocity );

            loop.sleep();
        }
    }
    port->drive( port_steering, 0.0 );
    starboard->drive( port_steering, 0.0 );
    stern->drive( port_steering, 0.0 );
}

void Motion::handleLock( void )
{
    // Should only get here from PAUSE
    // so saved_mode tells us what system is active
    switch(saved_mode)
    {
        case platform_motion_msgs::SelectMotionMode::Request::MODE_HOME:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE:
            driveToLock();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT:
            primePVT();
            waitForLockCond();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK:
            // nothing to do, already here
            break;
        default:
            ROS_ERROR("Unexpected state on lock entry: %s", motion_mode_string(motion_mode).c_str());
            break;
     }
}

void Motion::waitForLockCond( void )
{
    boost::unique_lock<boost::mutex> l(lock_state_mutex_);
    if( !lock_state_cond_.timed_wait( l, boost::posix_time::seconds(30) ) )
        ROS_ERROR( "WaitForLockCond timed out" );
}

void Motion::handleUnlock( void )
{
    // Should only get here through PAUSE
    // so saved_mode tells us what system is active
    switch(saved_mode)
    {
        case platform_motion_msgs::SelectMotionMode::Request::MODE_HOME:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_JOYSTICK:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_TWIST:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_SERVO:
        case platform_motion_msgs::SelectMotionMode::Request::MODE_ENABLE:
            driveToUnLock();
            break;
        case platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT:
            primePVT();
            waitForLockCond();
            break;
        default:
            ROS_ERROR("Unexpected state on lock entry: %s", motion_mode_string(motion_mode).c_str());
            break;
     }
}

bool Motion::targetReached( void )
{
    return ((port->steering.getStatusWord() & STATUS_TARGET_REACHED) == STATUS_TARGET_REACHED) &&
           ((starboard->steering.getStatusWord() & STATUS_TARGET_REACHED) == STATUS_TARGET_REACHED) &&
           ((stern->steering.getStatusWord() & STATUS_TARGET_REACHED) == STATUS_TARGET_REACHED);
}

void Motion::driveToLock( void )
{
    int timeout=300;
    do
    {
        port->drive( portSteeringLock_, 0.0 );
        starboard->drive( starboardSteeringLock_, 0.0 );
        stern->drive( sternSteeringLock_, 0.0 );
        ros::Duration(0.1).sleep();
        timeout--;
    }
    while( timeout>0 && !targetReached() && ros::ok());
}

void Motion::driveToUnLock( void )
{
    int timeout=300;
    do
    {
        port->drive( 0.0, 0.0 );
        starboard->drive( 0.0, 0.0 );
        stern->drive( 0.0, 0.0 );
        ros::Duration(0.1).sleep();
        timeout--;
    }
    while( timeout>0 && !targetReached() && ros::ok());
    // Move back to PAUSE
    motion_mode=platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE;
}

static double clamp(double angle, double min, double max)
{
    if ( angle > max ) { angle = max; }
    if ( angle < min ) { angle = min; }
    return angle;
}


void Motion::handleTwist(const geometry_msgs::Twist::ConstPtr twist)
{
    Eigen::Vector2d body_vel( twist->linear.x, twist->linear.y );
    double body_omega = twist->angular.z;
    double stern_wheel_speed, starboard_wheel_speed, port_wheel_speed;
    double stern_steering_angle, starboard_steering_angle, port_steering_angle;

    ROS_DEBUG("Twist linear.x: %2.5f , linear.y: %2.5f, angular.z: %2.5f", twist->linear.x, twist->linear.y, twist->angular.z);
    //get wheel angles, if they are outside allowable limits, clamp those values,
    //to return wheelpod to proper limits, otherwise wheelpod drive will flip the wheel
    //angles and you will be sad
    stern->getPosition(&stern_steering_angle, NULL, NULL, NULL);
    stern_steering_angle = clamp(stern_steering_angle, -M_PI_2, M_PI_2);
    computePod(body_vel, body_omega, body_pt, stern_pos_.head(2),
                &stern_steering_angle, &stern_wheel_speed);

    port->getPosition(&port_steering_angle, NULL, NULL, NULL);
    port_steering_angle = clamp(port_steering_angle, -M_PI_2, M_PI_2);
    computePod(body_vel, body_omega, body_pt, port_pos_.head(2),
                &port_steering_angle, &port_wheel_speed);

    starboard->getPosition(&starboard_steering_angle, NULL, NULL, NULL);
    starboard_steering_angle = clamp(starboard_steering_angle, -M_PI_2, M_PI_2);
    computePod(body_vel, body_omega, body_pt, starboard_pos_.head(2),
                &starboard_steering_angle, &starboard_wheel_speed);
    
    ROS_DEBUG("Drive port to angle: %2.5f , vel: %2.5f", port_steering_angle, port_wheel_speed);
    ROS_DEBUG("Drive starboard to angle: %2.5f , vel: %2.5f", starboard_steering_angle, starboard_wheel_speed);
    ROS_DEBUG("Drive stern to angle: %2.5f , vel: %2.5f", stern_steering_angle, stern_wheel_speed);
    
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    if( motion_mode != platform_motion_msgs::SelectMotionModeRequest::MODE_PAUSE )
    {
        port->drive(port_steering_angle, port_wheel_speed);
        starboard->drive(starboard_steering_angle, -starboard_wheel_speed);
        stern->drive(stern_steering_angle, -stern_wheel_speed);
    }
    else
    {
        ROS_ERROR("Attempted twist during pause.");
    }
}

int Motion::computePod(Eigen::Vector2d body_vel, double body_omega, Eigen::Vector2d body_pt,
        Eigen::Vector2d pod_pos, double *steering, double *speed)
{
    tf::StampedTransform pod_tf;
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
    if(pods_enabled == true && motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_HOME)
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
        const char * reason=pods_enabled ? "Not in motion mode MODE_HOME" : "Pods not enabled";
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
    (void) node;
    ROS_INFO("homed carousel");
    home_carousel_action_server.setSucceeded(home_carousel_result);  
}

bool Motion::ready(void)
{
    return port->ready() && starboard->ready() && stern->ready() && carousel->ready();
}

bool Motion::handleEnablePods(bool en, bool *result)
{
    bool notified=false;
    if(en==pods_enabled){
        ROS_DEBUG("Pods already in state %s", pods_enabled?"enabled":"disabled");
    }
    boost::unique_lock<boost::mutex> lock(enable_pods_mutex);
    boost::system_time now=boost::get_system_time();
    enable_pods(en);
    notified = enable_pods_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    if(!notified)
    {
        if( (port->enabled() == en) &&
            (starboard->enabled() == en) &&
            (stern->enabled() == en) )
        {
            pods_enabled = en;
            // not really, but successful anyway
            notified=true;
        }
        else
        {
            ROS_ERROR("Wheel pods enable unsuccessful time out");
        }
    }
    if(result) *result = pods_enabled;
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
    (void) node;
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

bool Motion::gpioServiceCallback(platform_motion_msgs::GPIOService::Request &req,
                                 platform_motion_msgs::GPIOService::Response &resp)
{
    if((unsigned long)req.gpio.servo_id == carousel->node_id) {
        boost::unique_lock<boost::mutex> lock(CAN_mutex);
        uint16_t current = carousel->getOutputPins();
        uint16_t set   = req.gpio.pin_mask & ( req.gpio.new_pin_states & ~current);
        uint16_t clear = req.gpio.pin_mask & (~req.gpio.new_pin_states &  current);
        CANOpen::PDOCallbackObject
            gpiocb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::PDOCallbackObject::CallbackFunction>(&Motion::gpioCompleteCallback));
        carousel->output(set, clear, gpiocb);
    } else {
            ROS_ERROR("Unknown servo_id in GPIO: %d", req.gpio.servo_id);
            return false;
    }
    bool notified;
    boost::unique_lock<boost::mutex> lock(gpio_service_mutex);
    boost::system_time now=boost::get_system_time();
    notified = gpio_service_cond.timed_wait(lock, now+boost::posix_time::milliseconds(100));
    if(!notified)
    {
        ROS_ERROR("GPIO service timeout");
    }
    resp.stamp = gpio_sent_timestamp;
    return notified;
}

void Motion::gpioCompleteCallback(CANOpen::PDO &pdo)
{
    boost::unique_lock<boost::mutex> lock(gpio_service_mutex);
    gpio_sent_timestamp = ros::Time::now();
    gpio_service_cond.notify_all();
}


void Motion::syncCallback(CANOpen::SYNC &sync)
{
    (void) sync;
    // reset the pv_counter to wait for all the servos to report status
    pv_counter=7;

    // reset the more data sent flag so we can send more data when needed
    this->moreDataSent = false;

    // increment the debug print counter
    m_debugPrintCounter++;

    // check to see if we're ready to start a coordinated pvt move
    // assuming we're in a mode that uses pvt moves..
    if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT )
    {
        if(port->getNeedsToStart() &&
           starboard->getNeedsToStart() &&
           stern->getNeedsToStart())
        {
            // if all the wheel pods are ready to go, tell them to start
            // moving. this means they should all start at the same time!
            ROS_DEBUG("Start moving %d %d %d %d %d %d %d %d %d %d %d %d",
                    port->wheel.getPvtBufferDepth(),
                    port->wheel.getPvtActive(),
                    port->steering.getPvtBufferDepth(),
                    port->steering.getPvtActive(),
                    starboard->wheel.getPvtBufferDepth(),
                    starboard->wheel.getPvtActive(),
                    starboard->steering.getPvtBufferDepth(),
                    starboard->steering.getPvtActive(),
                    stern->wheel.getPvtBufferDepth(),
                    stern->wheel.getPvtActive(),
                    stern->steering.getPvtBufferDepth(),
                    stern->steering.getPvtActive()
                    );
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
    (void) level;

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
    (void) event;

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

platform_motion_msgs::Knot
transformKnot( Eigen::Quaterniond rotation,
               Eigen::Vector3d translation,
               platform_motion_msgs::Knot k)
{
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position, kvelocity, komega;

    tf::quaternionMsgToEigen( k.pose.orientation, orientation );
    tf::pointMsgToEigen(k.pose.position, position );
    tf::vectorMsgToEigen( k.twist.linear, kvelocity );
    tf::vectorMsgToEigen( k.twist.angular, komega );

    orientation = orientation*rotation;
    position = translation + rotation*position;
    kvelocity = rotation*kvelocity;
    komega = rotation*komega;

    tf::quaternionEigenToMsg( orientation, k.pose.orientation );
    tf::pointEigenToMsg( position, k.pose.position );
    tf::vectorEigenToMsg( kvelocity, k.twist.linear );
    tf::vectorEigenToMsg( komega, k.twist.angular );
    return k;
}

void Motion::plannedPathCallback(const platform_motion_msgs::Path::ConstPtr path)
{
    if(motion_mode != platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT)
    {
        // if the planner hasn't been put in charge, don't listen to it
        return;
    }

    if( path->knots.size() < 2 )
    {
        ROS_ERROR( "Path too short: %ld", path->knots.size() );
        return;
    }

    // asking for time 0 asks for the most recent transform
    ros::Time most_recent(0);

    /***************************************************
     * 4 things we might want to do:
     * Absolute path in "odom" appended to path: path header != 0, knot[0] == 0
     * Absolute path in "odom" replaces path: path header !=0 , knot[0] != 0
     * Relative to body frame appended to path: path header == 0, knot[0]==0
     * Relative to body frame replaces path: path header == 0, knot[0] != 0
     *
     * Path interpretation strategy
     *
     * if path->knots[0].header.stamp != 0:
     *     drop all knots in plannedPath with stamp>=knots[0].header.stamp
     *
     * if path->knots[0].header.stamp == 0:
     *     if plannedPath.size() == 0:
     *         T = [ "odom" => "base link" ]
     *     else
     *         T = [ "odom" => plannedPath.back().pose ]
     * else
     *     T = [path->header => "odom"]
     *
     * append T*knot to plannedPath for all knots in path->knots
     *
     * In the relative case, the first point should be an identity motion
     * and the second point should be the first move. This allows a time
     * interval to be computed for this segment
     */
    ros::Time startTime;
    if( path->knots[0].header.stamp != ros::Time(0) )
    {
        // knots have non-zero time, remove any segments after that time
        //
        // take the new path and insert it into the plannedPath
        // list in the right spot.
        // first, remove everything in the list that is now made obsolete
        // by the new planned path.
        startTime = path->knots[0].header.stamp;
        std::unique_lock<std::mutex> lck(path_mutex_);
        plannedPath.remove_if(
                [startTime](platform_motion_msgs::Knot p)
                {
                return p.header.stamp >= startTime;
                }
                );
        lck.unlock();
    }
    else
    {
        if(plannedPath.size() == 0 )
        {
            // appending, but nothing to append to, start now
            startTime = ros::Time::now();
        }
        else
        {
            // appending, first point is an identity so use timestamp from
            // second
            startTime = plannedPath.back().header.stamp +
                (path->knots[1].header.stamp - path->knots[0].header.stamp);
        }
    }
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation, velocity, omega;
    if(path->header.stamp == ros::Time(0))
    {
        // 0 means relative to after whatever is going on as a relative path
        if(plannedPath.size() == 0 )
        {
            // No existing path, start at present position
            ROS_DEBUG("Start at present position");
            tf::StampedTransform initial;
            try {
                listener.lookupTransform("odom", "base_link", most_recent, initial );
            } catch( tf::TransformException ex) {
                ROS_ERROR("Error looking up %s: %s", "current robot pose", ex.what());
                return;
            }
            tf::quaternionTFToEigen( initial.getRotation(), rotation );
            tf::vectorTFToEigen( initial.getOrigin(), translation );
            velocity.setZero();
            omega.setZero();
        }
        else if(plannedPath.size() > 0)
        {
            // append at end of present path
            ROS_DEBUG("Append to existing path");
            platform_motion_msgs::Knot k = plannedPath.back();
            tf::quaternionMsgToEigen( k.pose.orientation, rotation );
            tf::pointMsgToEigen( k.pose.position, translation );
            tf::vectorMsgToEigen( k.twist.linear, velocity );
            tf::vectorMsgToEigen( k.twist.angular, omega );
        }
    }
    else
    {
        // non-zero means this is a valid frame transform which should be looked
        // up
        tf::StampedTransform T;
        try {
            listener.lookupTransform(path->header.frame_id, "odom", path->header.stamp, T );
        } catch( tf::TransformException ex) {
            ROS_ERROR("Error looking up %s->%s: %s", path->header.frame_id.c_str(), "odom", ex.what());
            return;
        }
        tf::quaternionTFToEigen( T.getRotation(), rotation );
        tf::vectorTFToEigen( T.getOrigin(), translation );
        velocity.setZero();
        omega.setZero();
    }

    ros::Duration dt = startTime - path->knots[0].header.stamp;

    std::unique_lock<std::mutex> lck(path_mutex_);
    for( auto k : path->knots )
    {
        // skip first point of relative path, it exists only to
        // set initial time interval and should be identical to the
        // the one before it.
        if(plannedPath.size()>0 && k.header.stamp == ros::Time(0))
            continue;
        k = transformKnot( rotation, translation, k);
        k.header.stamp += dt;
        k.header.frame_id = "odom";
        plannedPath.push_back(k);
    }
    lck.unlock();

    // publish stitched path for visualization
    if( stitchedPath_pub_.getNumSubscribers() > 0 )
    {
        platform_motion_msgs::Path stitched;
        stitched.knots.insert(stitched.knots.end(), plannedPath.begin(), plannedPath.end());
        stitched.header.frame_id="odom";
        stitched.header.stamp = ros::Time::now();
        stitched.header.seq = path->header.seq;
        stitchedPath_pub_.publish( stitched );
    }

    if( plannedPath.size()>0 && firstSegment_.time >= secondSegment_.time)
    {
        newPathReady = true;
    }

    primePVT();
}

void Motion::primePVT(void)
{
    // check to see if the wheel pods are out of data. if they are,
    // send a couple of segments to get things moving.
    if((port->getMinBufferDepth() < 1) ||
            (starboard->getMinBufferDepth() < 1) ||
            (stern->getMinBufferDepth() < 1))
    {
        ROS_DEBUG( "Restart needed" );
        restartPvt = true;
        sendPvtSegment();
    }

}

void Motion::moreDataNeededCallback(CANOpen::DS301 &node)
{
    (void) node;
    // only send more data once per sync pulse. this way we send data to
    // all the servos as soon as one says it needs more
    int depth = static_cast<CANOpen::CopleyServo&>(node).getPvtBufferDepth();
    ROS_DEBUG("More data needed %ld: %d", node.node_id, depth);
    if(!this->moreDataSent)
    {
        // set the flag so we don't send data for every callback call this
        // sync fame
        this->moreDataSent = true;

        // send the next segment
        sendPvtSegment();
        depth++;
        for(;depth<3;depth++)
        {
            ROS_DEBUG("Send priming segment");
            sendPvtSegment();
            restartPvt = false;
        }
    }
}

void Motion::errorCallback(CANOpen::DS301 &node)
{
    // handle a pvt related error here!
    // for now, just print out the error if we're in pvt mode...
    if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT )
    {
        ROS_ERROR("Servo with id %lu had the following error:%s",
                node.node_id,
                static_cast<CANOpen::CopleyServo*>(&node)->getLastError().c_str());
    }
}

bool Motion::pathToBody()
{
    ROS_DEBUG("pathToBody enter");

    firstSegment_ = secondSegment_;

    if( plannedPath.size() > 1 )
    {
        ROS_DEBUG("move to next segment");
        std::unique_lock<std::mutex> lck(path_mutex_);
        auto knots = plannedPath.begin();
        platform_motion_msgs::Knot previous = *knots++;
        platform_motion_msgs::Knot current = *knots++;
        platform_motion_msgs::Knot next;
        if(plannedPath.size() > 2)
            next = *knots++;
        else
        {
            next = current;
            next.header.stamp += ros::Duration(0.25);
        }
        plannedPath.pop_front();
        lck.unlock();

        if(   !pathToPod(previous, current, next, port_pos_, lastSegmentSent_.port.steeringAngle, &secondSegment_.port)
           || !pathToPod(previous, current, next, starboard_pos_, lastSegmentSent_.starboard.steeringAngle, &secondSegment_.starboard)
           || !pathToPod(previous, current, next, stern_pos_, lastSegmentSent_.stern.steeringAngle, &secondSegment_.stern) )
        {
            plannedPath.clear();
            return false;
        }
        else
        {
            secondSegment_.time = current.header.stamp;
            ROS_DEBUG("%d: port: \nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", current.header.seq, secondSegment_.port.steeringAngle, secondSegment_.port.steeringVelocity, secondSegment_.port.wheelDistance, secondSegment_.port.wheelVelocity, secondSegment_.port.duration);
            ROS_DEBUG("%d: starboard: \nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", current.header.seq, secondSegment_.starboard.steeringAngle, secondSegment_.starboard.steeringVelocity, secondSegment_.starboard.wheelDistance, secondSegment_.starboard.wheelVelocity, secondSegment_.starboard.duration);
            ROS_DEBUG("%d: stern: \nsteeringAngle: %f, steeringSpeed: %f, wheelDistance: %f, wheelVelocity: %f, duration: %f", current.header.seq, secondSegment_.stern.steeringAngle, secondSegment_.stern.steeringVelocity, secondSegment_.stern.wheelDistance, secondSegment_.stern.wheelVelocity, secondSegment_.stern.duration);
            ROS_DEBUG("first: %f second: %f", firstSegment_.time.toSec(), secondSegment_.time.toSec());
        }
    }
    else
    {
        plannedPath.clear();
        ROS_DEBUG("Finished path");
    }
    checkSegmentAcceleration();
    ROS_DEBUG("pathToBody exit");
    return true;
}

bool Motion::checkSegmentAcceleration()
{
    double dt = (secondSegment_.time - firstSegment_.time).toSec();
    typedef std::tuple<std::string, PodSegment, PodSegment> segtuple;
    std::vector< segtuple > segs;

    segs.push_back( segtuple( std::string("port"), firstSegment_.port, secondSegment_.port ) );
    segs.push_back( segtuple( std::string("starboard"), firstSegment_.starboard, secondSegment_.starboard ) );
    segs.push_back( segtuple( std::string("stern"), firstSegment_.stern, secondSegment_.stern ) );

    bool ok=true;
    for( auto seg : segs )
    {
        std::string name;
        PodSegment first, second;
        std::tie( name, first, second )=seg;
        if( fabs(second.wheelVelocity - first.wheelVelocity) > maxWheelAcceleration_*dt )
        {
            ROS_ERROR_STREAM("Excessive wheel acceleration for " << name << " :" << first << " - " << second);
            ok=false;
        }
        if( fabs(second.steeringAngle - first.steeringAngle) > maxSteeringVelocity_*dt )
        {
            ROS_ERROR_STREAM("Excessive steering change for " << name << " :" << first << " - " << second);
            ok=false;
        }
    }
    return ok;
}

double Motion::computeSafeInterval(double dt)
{
    typedef std::tuple<std::string, PodSegment, PodSegment> segtuple;
    std::vector< segtuple > segs;

    segs.push_back( segtuple( std::string("port"), firstSegment_.port, secondSegment_.port ) );
    segs.push_back( segtuple( std::string("starboard"), firstSegment_.starboard, secondSegment_.starboard ) );
    segs.push_back( segtuple( std::string("stern"), firstSegment_.stern, secondSegment_.stern ) );

    for( auto seg : segs )
    {
        std::string name;
        PodSegment first, second;
        std::tie( name, first, second )=seg;
        dt = std::max( dt, fabs(second.wheelVelocity - first.wheelVelocity)/maxWheelAcceleration_ );
        dt = std::max( dt, fabs(second.steeringAngle - first.steeringAngle)/maxSteeringVelocity_ );
    }
    return dt;
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

bool limit_steering_angle(double *phi)
{
    if(fabs(*phi+M_PI_2)<0.001)
    {
        *phi = -M_PI_2;
    }
    else if(fabs(*phi-M_PI_2)<0.001)
    {
        *phi = M_PI_2;
    }
    else if(fabs(*phi)>M_PI_2)
    {
        ROS_DEBUG( "Flipping velocity for smaller steering move" );
        *phi = unwrap(*phi+M_PI);
        return true;
    }
    return false;
}

bool Motion::pathToPod(platform_motion_msgs::Knot &previous, platform_motion_msgs::Knot &current, platform_motion_msgs::Knot &next, Eigen::Vector3d pod_pos, double &lastValidSteeringAngle, PodSegment* retval)
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

    retval->duration = dt_next;

    Eigen::Vector2d Vpod = podVelocity(current, pod_pos);

    double ksidot = Vpod.norm();

    if(ksidot<0.001)
    {
        // if wheel velocity is small, all other calculations are not possible
        retval->steeringAngle = lastValidSteeringAngle;
        retval->steeringVelocity = 0.0;
        retval->wheelDistance = 0.0;
        retval->wheelVelocity = 0.0;

        return true;
    }

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

    double phi = unwrap(atan2(Vpod[1], Vpod[0]) - currentTheta);
    if( limit_steering_angle(&phi) )
    {
        ksidot *= -1;
        dksi *= -1;
    }
    
    Eigen::Vector2d Vpod_next = podVelocity(next, pod_pos);
    double phi_next;
    if( Vpod_next.norm() > 0.001 )
    {
        phi_next = unwrap(atan2(Vpod_next[1], Vpod_next[0])-nextTheta);
    }
    else
    {
        phi_next = phi;
    }
    limit_steering_angle(&phi_next);

    double phidot = (phi_next - phi)/dt_next;

    if( isnan(phi) || isnan(phidot) ||
        isnan(dksi) || isnan(ksidot) )
    {
        ROS_ERROR( "Bad segments" );
        ROS_ERROR( "phi: %f phidot: %f dksi: %f ksidot: %f", phi, phidot, dksi, ksidot );
        ROS_ERROR_STREAM( "prev: " << previous );
        ROS_ERROR_STREAM( "current: " << current );
        ROS_ERROR_STREAM( "next: " << next );
        return false;
    }

    // we've actually computed everything now! stick the results into retval!
    retval->steeringAngle = phi;
    retval->steeringVelocity = phidot;
    retval->wheelDistance = dksi;
    retval->wheelVelocity = ksidot;

    // update last valid steering angle in case we hit another zero velocity path.
    lastValidSteeringAngle = phi;

    return true;
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

PodSegment interpolatePodSegment(const PodSegment &first, const PodSegment &second, double dadt, double alphaNow, double alphaLast)
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
    retval.port = interpolatePodSegment( first.port, second.port, dadt, alphaNow, alphaLast);
    retval.starboard = interpolatePodSegment( first.starboard, second.starboard, dadt, alphaNow, alphaLast);
    retval.stern = interpolatePodSegment( first.stern, second.stern, dadt, alphaNow, alphaLast);

    return retval;
}

void Motion::setLastSegmentToCurrent( void )
{
    lastSegmentSent_.port.duration = 0.25;
    lastSegmentSent_.starboard.duration = 0.0;
    lastSegmentSent_.stern.duration = 0.0;
    port->getPosition(&lastSegmentSent_.port.steeringAngle, &lastSegmentSent_.port.steeringVelocity,
            nullptr, &lastSegmentSent_.port.wheelVelocity);
    starboard->getPosition(&lastSegmentSent_.starboard.steeringAngle, &lastSegmentSent_.starboard.steeringVelocity,
            nullptr, &lastSegmentSent_.starboard.wheelVelocity);
    stern->getPosition(&lastSegmentSent_.stern.steeringAngle, &lastSegmentSent_.stern.steeringVelocity,
            nullptr, &lastSegmentSent_.stern.wheelVelocity);
    lastSegmentSent_.time = ros::Time::now();
}

void Motion::sendPvtSegment()
{
    // for now, only the planner should be sending pvt segments.
    // if the planner isn't in charge, don't send segments, including
    // the stop segments. this will keep the zero velocity segments from
    // stomping all over other commands from other sources.
    // if the joystick and visual servo ever use pvt mode, this will
    // have to change.
    if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PLANNER_PVT )
    {
        if( newPathReady )
        {
            if( lastSegmentSent_.time == ros::Time(0) )
            {
                setLastSegmentToCurrent();
            }

            // pathToBody() move this to firstSegment_
            secondSegment_ = lastSegmentSent_;
            secondSegment_.time = plannedPath.front().header.stamp;

            if(pathToBody())
            {

                // reset dt for first, this is computed by pathToBody for all future
                // segments
                double dt = (secondSegment_.time - firstSegment_.time).toSec();
                firstSegment_.port.duration = dt;
                firstSegment_.starboard.duration = dt;
                firstSegment_.stern.duration = dt;

                // set time so now works out to first, triggers
                // interpolation
                lastSegmentSent_.time=firstSegment_.time-ros::Duration(lastSegmentSent_.port.duration);
            }
            newPathReady = false;
        }

        if( secondSegment_.time>firstSegment_.time)
        {
            // send the next pvt segment to all the wheelpods.
            // if we've got some path to follow, follow it.
            // get the next two path segments out of the path
            ros::Time now=lastSegmentSent_.time+ros::Duration(lastSegmentSent_.port.duration);

            ROS_DEBUG_STREAM("first: " << firstSegment_.time << " second: " << secondSegment_.time << " now: " << now);

            BodySegment newSegment;

            if(now < firstSegment_.time)
            {
                ROS_DEBUG("now too old, sending first blindly");

                // if the last segment we sent was before this path, just send
                // the first segment. that's all we can really do.
                newSegment = firstSegment_;

            }
            else if(now == firstSegment_.time)
            {
                ROS_DEBUG("first segment reached exactly");

                // if the last segment we sent was before this path, just send
                // the first segment. that's all we can really do.
                newSegment = firstSegment_;

            }
            else if(now>secondSegment_.time)
            {
                ROS_ERROR("past second");
                plannedPath.clear();
                firstSegment_ = secondSegment_;
                pvtToZero();
            }
            else
            {
                if(lastSegmentSent_.time == firstSegment_.time && now == secondSegment_.time) {
                    ROS_DEBUG("No interpolation needed");
                    newSegment = secondSegment_;
                    pathToBody( );
                } else {
                    // compute the distance with the fancy cubic interpolation thing.
                    ROS_DEBUG("interpolating to now");
                    newSegment = interpolatePodSegments(firstSegment_, secondSegment_, lastSegmentSent_, now);
                    if( now == secondSegment_.time )
                        pathToBody( );
                }
                if((secondSegment_.time - now).toSec() < 0.255)
                {
                    ROS_DEBUG("interpolated close enough, duration to second");

                    double duration = (secondSegment_.time - now).toSec();

                    newSegment.port.duration = duration;
                    newSegment.starboard.duration = duration;
                    newSegment.stern.duration = duration;
                }
                else
                {
                    ROS_DEBUG("computing partial duration");

                    // send a fixed time step
                    // this prevents us from having a strange tiny duration at the end
                    double numSteps = ceil((secondSegment_.time - firstSegment_.time).toSec() / 0.255);
                    double step = (secondSegment_.time - firstSegment_.time).toSec() / numSteps;

                    // the next segment is step away from the last one in time.
                    newSegment.port.duration = step;
                    newSegment.starboard.duration = step;
                    newSegment.stern.duration = step;

                }
            }

            newSegment.time = now;

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

            lastSegmentSent_ = newSegment;
        }
        else
        {
            ROS_DEBUG("out of path segments, sending zeros");
            pvtToZero();
        }

    }
    else if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE )
    {
        if( plannedPath.size() > 0 )
        {
            ROS_INFO( "Clearing path in pause" );
            plannedPath.clear();
        }
        pvtToZero();
    }
    else if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_LOCK )
    {
        if( lastSegmentSent_.port.wheelVelocity > 0 ||
            lastSegmentSent_.starboard.wheelVelocity > 0 ||
            lastSegmentSent_.stern.wheelVelocity > 0 )
        {
            pvtToZero();
        }
        else
        {
            pvtToLock();
        }
    }
    else if( motion_mode == platform_motion_msgs::SelectMotionMode::Request::MODE_UNLOCK )
    {
        pvtToUnlock();
    }
    else
    {
        ROS_ERROR( "Not sending PVT from mode %s", motion_mode_string( motion_mode ).c_str() );
    }
}

double decelWheel( double xdot, double a, double dt, double *dx )
{
    double s=fabs(xdot);
    s = std::max(0.0, s-a*dt);
    double xdotprime = copysign( s, xdot );
    *dx = dt*(xdot+xdotprime)/2.;
    return xdotprime;
}

double decelSteering( double thetadot, double a, double dt, double *theta )
{
    double s=fabs(thetadot);
    s = std::max(0.0, s-a*dt);
    double thetadotprime = copysign( s, thetadot );
    *theta += dt*(thetadot + thetadotprime)/2.;
    return thetadotprime;
}

void Motion::pvtToZero( void )
{
    lastSegmentSent_.time += ros::Duration(0.250);
    lastSegmentSent_.port.duration = 0.250;
    lastSegmentSent_.starboard.duration = 0.250;
    lastSegmentSent_.stern.duration = 0.250;
    ROS_DEBUG( "lastSegmentSent_.port.wheelVelocity: %f", lastSegmentSent_.port.wheelVelocity);
    lastSegmentSent_.port.wheelVelocity = decelWheel(lastSegmentSent_.port.wheelVelocity, planToZeroDecel_, 0.25, &lastSegmentSent_.port.wheelDistance );
    lastSegmentSent_.port.steeringVelocity = decelSteering(lastSegmentSent_.port.steeringVelocity, steeringAccel_, 0.25, &lastSegmentSent_.port.steeringAngle );
    lastSegmentSent_.starboard.wheelVelocity = decelWheel(lastSegmentSent_.starboard.wheelVelocity, planToZeroDecel_, 0.25, &lastSegmentSent_.starboard.wheelDistance );
    lastSegmentSent_.starboard.steeringVelocity = decelSteering(lastSegmentSent_.starboard.steeringVelocity, steeringAccel_, 0.25, &lastSegmentSent_.starboard.steeringAngle );
    lastSegmentSent_.stern.wheelVelocity = decelWheel(lastSegmentSent_.stern.wheelVelocity, planToZeroDecel_, 0.25, &lastSegmentSent_.stern.wheelDistance );
    lastSegmentSent_.stern.steeringVelocity = decelSteering(lastSegmentSent_.stern.steeringVelocity, steeringAccel_, 0.25, &lastSegmentSent_.stern.steeringAngle );
    port->move(lastSegmentSent_.port);
    starboard->move(lastSegmentSent_.starboard);
    stern->move(lastSegmentSent_.stern);
    ROS_DEBUG( "Zeroing steering: %f %f %f %f %f %f",
            lastSegmentSent_.port.steeringAngle, lastSegmentSent_.port.steeringVelocity,
            lastSegmentSent_.starboard.steeringAngle, lastSegmentSent_.starboard.steeringVelocity,
            lastSegmentSent_.stern.steeringAngle, lastSegmentSent_.stern.steeringVelocity
            );
    ROS_DEBUG( "Zeroing wheels: %f %f %f %f %f %f",
            lastSegmentSent_.port.wheelDistance, lastSegmentSent_.port.wheelVelocity,
            lastSegmentSent_.starboard.wheelDistance, lastSegmentSent_.starboard.wheelVelocity,
            lastSegmentSent_.stern.wheelDistance, lastSegmentSent_.stern.wheelVelocity
            );
}

bool Motion::setSteering( PodSegment &pod, double goal, double dt)
{
    double error = ( goal - pod.steeringAngle );
    if(fabs(error)<steeringTolerance_)
    {
        pod.steeringAngle = goal;
        pod.steeringVelocity = 0;
        return false;
    }
    double vprime;
    if(fabs(error)<=0.5*maxSteeringVelocity_*maxSteeringVelocity_/steeringAccel_)
    {
        // decel region
        vprime = copysign( sqrt(2.0*fabs(error)*steeringAccel_) , error );
    }
    else
    {
        vprime = copysign( std::min( maxSteeringVelocity_, fabs(pod.steeringVelocity) + dt*steeringAccel_ ), error );
    }
    double dtheta = dt*(pod.steeringVelocity + vprime)/2.;
    if( fabs(dtheta)<fabs(error) )
    {
        pod.steeringAngle += dtheta;
        pod.steeringVelocity = vprime;
    }
    else
    {
        pod.steeringAngle = goal;
        pod.steeringVelocity = 0;
    }
    return true;
}

void Motion::pvtToLock( void )
{
    lastSegmentSent_.time += ros::Duration(0.250);
    lastSegmentSent_.port.duration = 0.250;
    lastSegmentSent_.starboard.duration = 0.250;
    lastSegmentSent_.starboard.duration = 0.250;
    bool moving;
    moving = setSteering( lastSegmentSent_.port, portSteeringLock_, 0.25);
    moving |= setSteering( lastSegmentSent_.starboard, starboardSteeringLock_,  0.25);
    moving |= setSteering( lastSegmentSent_.stern, sternSteeringLock_, 0.25);
    if( !moving )
    {
        boost::unique_lock<boost::mutex> l(lock_state_mutex_);
        lock_state_cond_.notify_all();
    }
    port->move(lastSegmentSent_.port);
    starboard->move(lastSegmentSent_.starboard);
    stern->move(lastSegmentSent_.stern);
    ROS_DEBUG( "Locking: %f %f %f %f %f %f",
            lastSegmentSent_.port.steeringAngle, lastSegmentSent_.port.steeringVelocity,
            lastSegmentSent_.starboard.steeringAngle, lastSegmentSent_.starboard.steeringVelocity,
            lastSegmentSent_.stern.steeringAngle, lastSegmentSent_.stern.steeringVelocity
            );
}

void Motion::pvtToUnlock( void )
{
    lastSegmentSent_.time += ros::Duration(0.250);
    lastSegmentSent_.port.duration = 0.250;
    lastSegmentSent_.starboard.duration = 0.250;
    lastSegmentSent_.starboard.duration = 0.250;
    bool moving;
    moving = setSteering( lastSegmentSent_.port, 0.0, 0.25);
    moving |= setSteering( lastSegmentSent_.starboard, 0.0, 0.25);
    moving |= setSteering( lastSegmentSent_.stern, 0.0, .25);
    if( !moving )
    {
        motion_mode=platform_motion_msgs::SelectMotionMode::Request::MODE_PAUSE;
        platform_motion_msgs::SelectMotionMode::Response msg;
        msg.mode = motion_mode;
        motionMode_pub_.publish( msg );
        boost::unique_lock<boost::mutex> l(lock_state_mutex_);
        lock_state_cond_.notify_all();
    }
    port->move(lastSegmentSent_.port);
    starboard->move(lastSegmentSent_.starboard);
    stern->move(lastSegmentSent_.stern);
    ROS_DEBUG( "Unlocking: %f %f %f %f %f %f",
            lastSegmentSent_.port.steeringAngle, lastSegmentSent_.port.steeringVelocity,
            lastSegmentSent_.starboard.steeringAngle, lastSegmentSent_.starboard.steeringVelocity,
            lastSegmentSent_.stern.steeringAngle, lastSegmentSent_.stern.steeringVelocity
            );
}

}
