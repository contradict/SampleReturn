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
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion/HomeAction.h>
#include <platform_motion/Enable.h>
#include <platform_motion/PlatformParametersConfig.h>
#include <platform_motion/GPIO.h>
#include <platform_motion/ServoStatus.h>
#include <motion/wheelpod.h>

#include <motion/motion.h>

#define POLL_TIMEOUT_MS 10

namespace platform_motion {

Motion::Motion() :
    param_nh("~"),
    home_pods_action_server(nh_, "home_wheelpods", false),
    home_carousel_action_server(nh_, "home_carousel", false),
    CAN_fd(-1),
    gpio_enabled(false),
    carousel_setup(false),
    pods_enabled(false),
    desired_pod_state(false),
    carousel_enabled(false),
    desired_carousel_state(false),
    pv_counter(0),
    joint_seq(0)
{

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

    twist_sub = nh_.subscribe("twist", 2, &Motion::twistCallback, this);

    carousel_sub = nh_.subscribe("carousel", 2, &Motion::carouselCallback,
            this);

    gpio_sub = nh_.subscribe("gpio_write", 2, &Motion::gpioSubscriptionCallback,
            this);

    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("platform_joint_state", 1);

    gpio_pub = nh_.advertise<platform_motion::GPIO>("gpio_read", 1);

    battery_voltage_pub = nh_.advertise<std_msgs::Float64>("battery_voltage", 1);

    status_pub = nh_.advertise<ServoStatus>("status_word", 1);

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
                 large_steering_move
                ));
    port->setCallbacks(pvcb, pvcb, gpiocb);
    port->steering.setStatusCallback(stcb);
    port->wheel.setStatusCallback(stcb);

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
                 large_steering_move
                ));
    starboard->setCallbacks(pvcb, pvcb, gpiocb);
    starboard->steering.setStatusCallback(stcb);
    starboard->wheel.setStatusCallback(stcb);


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
                 large_steering_move
                ));
    stern->setCallbacks(pvcb, pvcb, gpiocb);
    stern->steering.setStatusCallback(stcb);
    stern->wheel.setStatusCallback(stcb);

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

void Motion::twistCallback(const geometry_msgs::Twist::ConstPtr twist)
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
    if(pods_enabled == true)
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
        ROS_INFO("Home pods goal aborted - not enabled");
        home_pods_result.homed.clear();
        home_pods_result.homed.push_back(false);
        home_pods_result.homed.push_back(false);
        home_pods_result.homed.push_back(false);
        home_pods_action_server.setAborted(home_pods_result, "Pods not enabled");
    }
}

void Motion::doHomeCarousel(void)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    if(carousel_enabled)
    {
        ROS_INFO("Home carousel goal accepted");
        home_carousel_action_server.acceptNewGoal();
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
    home_carousel_action_server.setSucceeded(home_carousel_result);  
}

bool Motion::ready(void)
{
    return port->ready() && starboard->ready() && stern->ready() && carousel->ready();
}

bool Motion::enableWheelPodsCallback(platform_motion::Enable::Request &req,
                                     platform_motion::Enable::Response &resp)
{
    bool notified=false;
    boost::unique_lock<boost::mutex> lock(enable_pods_mutex);
    boost::system_time now=boost::get_system_time();
    enable_pods(req.state);
    notified = enable_pods_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    resp.state = pods_enabled;
    return notified;
}

bool Motion::enableCarouselCallback(platform_motion::Enable::Request &req,
                                    platform_motion::Enable::Response &resp)
{
    bool notified=false;
    boost::unique_lock<boost::mutex> lock(enable_carousel_mutex);
    boost::system_time now=boost::get_system_time();
    enable_carousel(req.state);
    notified = enable_carousel_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    resp.state = carousel_enabled;
    return notified;
}


void Motion::podEnableStateChange(CANOpen::DS301 &node)
{
    if(enable_pods_count>0) --enable_pods_count;
    ROS_INFO("Enable report from %ld, count=%d", node.node_id, enable_pods_count);
    if(enable_pods_count == 0) {
        boost::unique_lock<boost::mutex> lock(enable_pods_mutex);
        enable_pods_cond.notify_all();
        pods_enabled = desired_pod_state;
    }
}

void Motion::carouselEnableStateChange(CANOpen::DS301 &node)
{
    ROS_INFO("Enable report from %ld, carousel", node.node_id);
    boost::unique_lock<boost::mutex> lock(enable_carousel_mutex);
    enable_carousel_cond.notify_all();
    carousel_enabled = desired_carousel_state;
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
    platform_motion::GPIO gpio;

    gpio.servo_id = svo.node_id;
    gpio.previous_pin_states = old_pins;
    gpio.new_pin_states = new_pins;
    gpio.pin_mask = 0xFFFF;
    gpio_pub.publish(gpio);
}

void Motion::gpioSubscriptionCallback(const platform_motion::GPIO::ConstPtr gpio)
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
    pv_counter=7;
    /*
    if(carousel_enabled)
        pv_counter += 1;
    if(pods_enabled)
        pv_counter += 6;
        */
    if(carousel->ready()) {
        if(!carousel_setup ) {
            // set trajectory jerk limit
            std::vector<uint8_t> data;
            uint32_t jerk_limit = rint(carousel_jerk_limit*carousel_encoder_counts/100);
            CANOpen::Transfer::pack((uint32_t)jerk_limit, data); // 100 counts/second^3
            carousel->writeObjectDictionary(0x2121, 0, data);
            // profile velocity during position move
            data.clear();
            int32_t profile_velocity=rint(carousel_profile_velocity*carousel_encoder_counts/0.1);
            CANOpen::Transfer::pack(profile_velocity, data); // 0.1 counts/sec
            carousel->writeObjectDictionary(0x6081, 0, data);
            // profile acceleration, used to slow down at the end of
            // an s-curve
            data.clear();
            int32_t profile_acceleration=rint(carousel_profile_acceleration*carousel_encoder_counts/10);
            CANOpen::Transfer::pack(profile_acceleration, data); // 10 counts/sec
            carousel->writeObjectDictionary(0x6083, 0, data);
            // motion profile type
            data.clear();
            CANOpen::Transfer::pack((int16_t)3, data); // s-curve
            carousel->writeObjectDictionary(0x6086, 0, data);
            carousel_setup=true;
        }
        if(!gpio_enabled) {
            carousel->inputPinPullups(0x0003);
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
        platform_motion::GPIO gpio;
        gpio.servo_id = carousel->node_id;
        gpio.previous_pin_states = carousel->getInputPins();
        gpio.new_pin_states = gpio.previous_pin_states;
        gpio.pin_mask = 0xFFFF;
        gpio_pub.publish(gpio);

        std_msgs::Float64 voltage;
        voltage.data = carousel->bus_voltage;
        battery_voltage_pub.publish(voltage);
    }

    ServoStatus status;
    uint16_t steering_status, wheel_status;
    port->getStatusWord( &steering_status, &wheel_status);
    status.servo_id=port->steering.node_id;
    status.status_word = steering_status;
    status_pub.publish(status);
    status.servo_id=port->wheel.node_id;
    status.status_word = wheel_status;
    status_pub.publish(status);

    starboard->getStatusWord( &steering_status, &wheel_status);
    status.servo_id=starboard->steering.node_id;
    status.status_word = steering_status;
    status_pub.publish(status);
    status.servo_id=starboard->wheel.node_id;
    status.status_word = wheel_status;
    status_pub.publish(status);

    stern->getStatusWord( &steering_status, &wheel_status);
    status.servo_id=stern->steering.node_id;
    status.status_word = steering_status;
    status_pub.publish(status);
    status.servo_id=stern->wheel.node_id;
    status.status_word = wheel_status;
    status_pub.publish(status);

    status.servo_id = carousel->node_id;
    status.status_word = carousel->getStatusWord();
    status_pub.publish(status);
}

void Motion::statusCallback(CANOpen::DS301 &node)
{
    CANOpen::CopleyServo *svo = static_cast<CANOpen::CopleyServo*>(&node);
    ServoStatus status;

    status.servo_id=svo->node_id;
    status.status_word = svo->getStatusWord();
    status_pub.publish(status);
}

}
