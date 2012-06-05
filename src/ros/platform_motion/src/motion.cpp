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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion/HomeAction.h>
#include <platform_motion/Enable.h>
#include <platform_motion/PlatformParametersConfig.h>
#include <platform_motion/GPIO.h>
#include <motion/wheelpod.h>

#include <motion/motion.h>

#include <motion/lmmin.h>

#define POLL_TIMEOUT_MS 10

namespace platform_motion {

struct lmmin_data {
    Eigen::Vector2d body_pt;
    Eigen::Vector2d port_pos, port_delta;
    double port_vel;
    Eigen::Vector2d stern_pos, stern_delta;
    double stern_vel;
    Eigen::Vector2d starboard_pos, starboard_delta;
    double starboard_vel;
};

void lmmin_evaluate(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info)
{
    const struct lmmin_data *data = reinterpret_cast<const struct lmmin_data *>(vdata);

    double x,y,theta;

    x=xytheta[0];
    y=xytheta[1];
    theta=xytheta[2];

    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta),  cos(theta);

    Eigen::Vector2d T(x, y);

    Eigen::Vector2d port_error = R*(data->port_pos - data->body_pt) + data->body_pt +
        T - (data->port_pos + data->port_delta);
    Eigen::Vector2d stern_error = R*(data->stern_pos - data->body_pt) + data->body_pt +
        T - (data->stern_pos + data->stern_delta);
    Eigen::Vector2d starboard_error = R*(data->starboard_pos - data->body_pt) + data->body_pt +
        T - (data->starboard_pos + data->starboard_delta);

    fvec[0] = port_error(0);
    fvec[1] = port_error(1);
    fvec[2] = stern_error(0);
    fvec[3] = stern_error(1);
    fvec[4] = starboard_error(0);
    fvec[5] = starboard_error(1);

    *info = 1;
}


Motion::Motion() :
    home_pods_action_server(nh_, "home_wheelpods", false),
    home_carousel_action_server(nh_, "home_carousel", false),
    CAN_fd(-1),
    gpio_enabled(false),
    pods_enabled(false),
    desired_pod_state(false),
    carousel_enabled(false),
    desired_carousel_state(false),
    last_starboard_wheel(0), last_port_wheel(0), last_stern_wheel(0),
    odom_count(1), odom_counter(1),
    port_vel_sum(0), starboard_vel_sum(0), stern_vel_sum(0),
    pv_counter(0),
    odom_position(Eigen::Vector2d::Zero()),
    odom_orientation(0),
    odom_frame_id(0),
    joint_seq(0)
{
    nh_.param("port_steering_id", port_steering_id, 4);
    nh_.param("port_wheel_id", port_wheel_id, 2);
    nh_.param("port_steering_min", port_steering_min, -M_PI_2);
    nh_.param("port_steering_max", port_steering_max,  M_PI_2);
    nh_.param("port_steering_offset", port_steering_offset,  0.0);

    nh_.param("starboard_steering_id", starboard_steering_id, 7);
    nh_.param("starboard_wheel_id", starboard_wheel_id, 6);
    nh_.param("starboard_steering_min", starboard_steering_min, -M_PI_2);
    nh_.param("starboard_steering_max", starboard_steering_max,  M_PI_2);
    nh_.param("starboard_steering_offset", starboard_steering_offset,  0.0);

    nh_.param("stern_steering_id", stern_steering_id, 3);
    nh_.param("stern_wheel_id", stern_wheel_id, 5);
    nh_.param("stern_steering_min", stern_steering_min, -M_PI_2);
    nh_.param("stern_steering_max", stern_steering_max,  M_PI_2);
    nh_.param("stern_steering_offset", stern_steering_offset,  0.0);

    nh_.param("carousel_id", carousel_id, 1);
    nh_.param("carousel_encoder_counts", carousel_encoder_counts, 4*2500);
    nh_.param("carousel_offset", carousel_offset, 0.0);
    nh_.param("sync_interval", sync_interval, 50000);

    nh_.param("wheel_diameter", wheel_diameter, 0.330);
    nh_.param("steering_encoder_counts", steering_encoder_counts, 4*2500);
    nh_.param("wheel_encoder_counts", wheel_encoder_counts, 4*2500);
    nh_.param("large_steering_move", large_steering_move, 30);

    nh_.param("robot_width", width, 1.3);
    nh_.param("robot_length", length, 1.125);
    nh_.param("center_pt_x", center_pt_x, 1.125);
    nh_.param("center_pt_y", center_pt_y, 0.0);

    nh_.param("CAN_channel", CAN_channel, 0);
    nh_.param("CAN_baud", CAN_baud, 1000000);

    nh_.param("enable_wait_timeout_ms", enable_wait_timeout, 150);

    port_pos << length, width/2;
    starboard_pos << length, -width/2;
    stern_pos << 0, 0;
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

    odometry_pub = nh_.advertise<nav_msgs::Odometry>("odometry", 1);

    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("platform_joint_state", 1);

    gpio_pub = nh_.advertise<platform_motion::GPIO>("gpio_read", 1);

    home_pods_action_server.registerGoalCallback(boost::bind(&Motion::doHomePods, this));
    home_carousel_action_server.registerGoalCallback(boost::bind(&Motion::doHomeCarousel, this));

    reconfigure_server.setCallback(boost::bind(&Motion::reconfigureCallback, this, _1, _2));

}

bool Motion::openBus(void)
{
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

    CANOpen::CopleyServo::InputChangeCallback
        gpiocb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::CopleyServo::InputChangeCallback::CallbackFunction>(&Motion::gpioCallback));

    carousel = std::tr1::shared_ptr<CANOpen::CopleyServo>(new
            CANOpen::CopleyServo(carousel_id, sync_interval, pbus));
    carousel->setInputCallback(gpiocb);

    CANOpen::DS301CallbackObject pvcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::pvCallback));
                          

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

    port->initialize();
    starboard->initialize();
    stern->initialize();
    carousel->initialize();

    return true;
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
    }
    open_lock.unlock();
    if( ! CAN_thread_run )
        return;
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
    ROS_INFO("Send initialize notification");
    ROS_INFO("Initialize all servos" );
}


void Motion::twistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    Eigen::Vector2d body_vel( twist->linear.x, twist->linear.y );
    double body_speed = body_vel.norm();
    double body_omega = twist->angular.z;
    Eigen::Vector2d center_pos;
    double kappa, phi;
    if(fabs(body_omega)>0) {
        if(body_speed>0) {
            Eigen::Vector2d center_dir;
            center_dir << -body_vel[1], body_vel[0];
            center_dir *= copysign(1.0/body_speed, body_omega);
            kappa = fabs(body_omega)/body_speed;
            center_pos = body_pt + center_dir/kappa;
        } else {
            kappa = 1./body_pt.norm();
            center_pos = body_pt;
        }
        phi = atan2(center_pos[1], center_pos[0]);
        v_stern = center_pos.norm()*body_omega/M_PI/wheel_diameter;
        v_port =
            (center_pos-port_pos).norm()*body_omega/M_PI/wheel_diameter;
        v_starboard =
            (center_pos-starboard_pos).norm()*body_omega/M_PI/wheel_diameter;
    } else {
        kappa = 0;
        phi = atan2(body_vel[1], body_vel[0])+M_PI_2;
        v_stern = v_starboard = v_port = body_speed/M_PI/wheel_diameter;
    }
    angle_stern = phi-M_PI_2;
    angle_port = M_PI_2 - atan2( sin(phi) - kappa*width/2,
                                      kappa*length - cos(phi)
                                    );
    angle_starboard = M_PI_2 - atan2( sin(phi) + kappa*width/2,
                                 kappa*length - cos(phi)
                               );
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    port->drive(angle_port, v_port);
    starboard->drive(angle_starboard, -v_starboard);
    stern->drive(angle_stern, -v_stern);
}



void Motion::doHomePods(void)
{
    home_pods_count = 3;
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    ROS_INFO("Home pods goal accepted");
    home_pods_action_server.acceptNewGoal();
    CANOpen::DS301CallbackObject
        hcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::homePodsComplete));
    port->home(hcb);
    starboard->home(hcb);
    stern->home(hcb);
}

void Motion::doHomeCarousel(void)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    ROS_INFO("Home carousel goal accepted");
    home_carousel_action_server.acceptNewGoal();
    CANOpen::DS301CallbackObject
        hcb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::homeCarouselComplete));
    carousel->home(hcb);
}

void Motion::homePodsComplete(CANOpen::DS301 &node)
{
    if(home_pods_count>0) home_pods_count--;
    if(home_pods_action_server.isActive()) {
        home_pods_feedback.home_count = home_pods_count;
        home_pods_action_server.publishFeedback(home_pods_feedback);
        if(home_pods_count == 0) {
            home_pods_action_server.setSucceeded(home_pods_result);  
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
    if( pods_enabled == req.state)
        return true;
    enable_pods_count=6;
    boost::system_time now=boost::get_system_time();
    boost::unique_lock<boost::mutex> lock(enable_pods_mutex);
    bool notified=false;
    enable_pods(req.state);
    while( ! pods_enabled==req.state )
        notified = enable_pods_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    if(notified)
        resp.state = pods_enabled;
    return notified;
}

void Motion::podEnableStateChange(CANOpen::DS301 &node)
{
    if(enable_pods_count>0) --enable_pods_count;
    ROS_INFO("Enable report, count=%d", enable_pods_count);
    if(enable_pods_count == 0) {
        enable_pods_cond.notify_one();
        pods_enabled = desired_pod_state;
    }
}

void Motion::carouselEnableStateChange(CANOpen::DS301 &node)
{
    enable_carousel_cond.notify_one();
    carousel_enabled = desired_carousel_state;
}

void Motion::enable_pods(bool state)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    ROS_INFO( "enable pods: %s",  state?"true":"false");

    CANOpen::DS301CallbackObject
        ecb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::podEnableStateChange));
    desired_pod_state = state;
    port->enable(state, ecb);
    starboard->enable(state, ecb);
    stern->enable(state, ecb);
}

bool Motion::enableCarouselCallback(platform_motion::Enable::Request &req,
                                    platform_motion::Enable::Response &resp)
{
    if(carousel_enabled == req.state)
        return true;
    boost::unique_lock<boost::mutex> lock(enable_carousel_mutex);
    bool notified=false;
    boost::system_time now=boost::get_system_time();
    enable_carousel(req.state);
    while( ! carousel_enabled==req.state )
        notified = enable_carousel_cond.timed_wait(lock, now+boost::posix_time::milliseconds(enable_wait_timeout));
    if(notified)
        resp.state = carousel_enabled;
    return notified;
}

void Motion::enable_carousel(bool state)
{
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    ROS_INFO( "enable carousel: %s",  state?"true":"false");

    CANOpen::DS301CallbackObject
        ecb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
            static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::carouselEnableStateChange));
    desired_carousel_state = state;
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

        port->getPosition(port_steering, port_steering_velocity,
                port_wheel, port_wheel_velocity);
        starboard->getPosition(starboard_steering, starboard_steering_velocity,
                starboard_wheel, starboard_wheel_velocity);
        stern->getPosition(stern_steering, stern_steering_velocity,
                stern_wheel, stern_wheel_velocity);

        port_vel_sum += port_wheel_velocity;
        starboard_vel_sum += starboard_wheel_velocity;
        stern_vel_sum += stern_wheel_velocity;

        if(--odom_counter>0) {
            return;
        } else {
            odom_counter = odom_count;
            port_wheel_velocity = port_vel_sum/odom_count*M_PI*wheel_diameter;
            port_vel_sum = 0;
            starboard_wheel_velocity =
                starboard_vel_sum/odom_count*M_PI*wheel_diameter;
            starboard_vel_sum = 0;
            stern_wheel_velocity = stern_vel_sum/odom_count*M_PI*wheel_diameter;
            stern_vel_sum = 0;
        }

        starboard_wheel *= -1;
        starboard_wheel_velocity *= -1;
        stern_wheel *= -1;
        stern_wheel_velocity *= -1;

        double port_delta, starboard_delta, stern_delta;

        port_delta = (port_wheel - last_port_wheel)*M_PI*wheel_diameter;
        last_port_wheel = port_wheel;
        starboard_delta = (starboard_wheel - last_starboard_wheel)*M_PI*wheel_diameter;
        last_starboard_wheel = starboard_wheel;
        stern_delta = (stern_wheel - last_stern_wheel)*M_PI*wheel_diameter;
        last_stern_wheel = stern_wheel;

        Eigen::Vector2d port_wheel_direction(cos(port_steering), sin(port_steering));
        Eigen::Vector2d stern_wheel_direction(cos(stern_steering), sin(stern_steering));
        Eigen::Vector2d starboard_wheel_direction(cos(starboard_steering), sin(starboard_steering));

        double max_abs_vel = std::max(fabs(port_wheel_velocity), fabs(starboard_wheel_velocity));
        max_abs_vel = std::max(max_abs_vel, fabs(stern_wheel_velocity));
        ROS_DEBUG( "vel: (%6.3f, %6.3f, %6.3f) max: %6.3f",
                port_wheel_velocity, starboard_wheel_velocity, stern_wheel_velocity,
                max_abs_vel);
        nav_msgs::Odometry odo;
        odo.header.frame_id = "odom";
        odo.child_frame_id = "base_link";
        odo.header.stamp = current_time;
        if(max_abs_vel<5e-2) {
            odo.twist.twist.linear.x = 0;
            odo.twist.twist.linear.y = 0;
            odo.twist.twist.linear.z = 0;
            odo.twist.twist.angular.x = 0;
            odo.twist.twist.angular.y = 0;
            odo.twist.twist.angular.z = 0;
            // xx
            odo.pose.covariance[0] = 1e-10;
            // yy
            odo.pose.covariance[7] = 1e-10;
            // xy
            odo.pose.covariance[1] = odo.pose.covariance[6] = 1e-10;
            // xyaw
            odo.pose.covariance[5] = odo.pose.covariance[30] = 1e-10;
            // yyaw
            odo.pose.covariance[11] = odo.pose.covariance[31] = 1e-10;
            // yaw*yaw
            odo.pose.covariance[35] = 1e-10;
            // vxvx
            odo.twist.covariance[0] = 1e-10;
            // vyvy
            odo.twist.covariance[7] = 1e-10;
            // vxvy
            odo.twist.covariance[1] = odo.pose.covariance[6] = 1e-10;
            // vxomegaz
            odo.twist.covariance[5] = odo.pose.covariance[30] = 1e-10;
            // vyomegaz
            odo.twist.covariance[11] = odo.pose.covariance[31] = 1e-10;
            // omegaz*omegaz
            odo.twist.covariance[35] = 1e-10;
        } else {
            struct lmmin_data data;
            data.body_pt = body_pt;
            data.port_pos = port_pos;
            data.port_delta = port_delta*port_wheel_direction;
            data.port_vel = port_wheel_velocity;
            data.stern_pos = stern_pos;
            data.stern_delta = stern_delta*stern_wheel_direction;
            data.stern_vel = stern_wheel_velocity;
            data.starboard_pos = starboard_pos;
            data.starboard_delta = starboard_delta*starboard_wheel_direction;
            data.starboard_vel = starboard_wheel_velocity;

            double xytheta[3] = {0.0, 0.0, 0.0};
            lm_status_struct status;
            lm_control_struct control = lm_control_double;
            control.printflags = 0;
            lmmin( 3, xytheta, 6, &data, lmmin_evaluate, &control, &status, NULL);

            ROS_DEBUG( "delta: (%6.4f, %6.4f, %6.4f)", port_delta, starboard_delta, stern_delta);

            if(status.info>4) {
                // minimization failed
                ROS_ERROR( "%s", (std::string("Minimization failed: ") +
                                    lm_infmsg[status.info]).c_str() );
            } else {
                Eigen::Matrix2d R;
                R << cos(odom_orientation), -sin(odom_orientation),
                  sin(odom_orientation),  cos(odom_orientation);
                odom_position += R*Eigen::Vector2d(xytheta[0], xytheta[1]);
                odom_orientation += xytheta[2];
                if(odom_orientation > 2*M_PI) odom_orientation -= 2*M_PI;
                if(odom_orientation < 0) odom_orientation += 2*M_PI;

                /*
                odo.twist.twist.linear.x = xythetavxvyomega[3];
                odo.twist.twist.linear.y = xythetavxvyomega[4];
                odo.twist.twist.linear.z = 0;
                odo.twist.twist.angular.x = 0;
                odo.twist.twist.angular.y = 0;
                odo.twist.twist.angular.z = xythetavxvyomega[5];
                */
            }
            // xx
            odo.pose.covariance[0] = 0.05;
            // yy
            odo.pose.covariance[7] = 0.05;
            // xy
            odo.pose.covariance[1] = odo.pose.covariance[6] = 0.005;
            // xyaw
            odo.pose.covariance[5] = odo.pose.covariance[30] = 0.001;
            // yyaw
            odo.pose.covariance[11] = odo.pose.covariance[31] = 0.001;
            // yaw*yaw
            odo.pose.covariance[35] =
                pow((2.*M_PI/((double)steering_encoder_counts)), 2);
            // vxvx
            odo.twist.covariance[0] = 10.0;
            // vyvy
            odo.twist.covariance[7] = 10.0;
            // vxvy
            odo.twist.covariance[1] = odo.pose.covariance[6] = 10.0;
            // vxomegaz
            odo.twist.covariance[5] = odo.pose.covariance[30] = 10.0;
            // vyomegaz
            odo.twist.covariance[11] = odo.pose.covariance[31] = 10.0;
            // omegaz*omegaz
            odo.twist.covariance[35] = 10.0;
        }
        odo.pose.covariance[14] = DBL_MAX;
        odo.pose.covariance[21] = DBL_MAX;
        odo.pose.covariance[28] = DBL_MAX;
        odo.twist.covariance[14] = DBL_MAX;
        odo.twist.covariance[21] = DBL_MAX;
        odo.twist.covariance[28] = DBL_MAX;
        odo.pose.pose.position.x = odom_position(0);
        odo.pose.pose.position.y = odom_position(1);
        odo.pose.pose.position.z = 0;
        geometry_msgs::Quaternion odom_quat =
            tf::createQuaternionMsgFromYaw(odom_orientation);
        odo.pose.pose.orientation = odom_quat;
        odometry_pub.publish(odo);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = odom_position(0);
        odom_trans.transform.translation.y = odom_position(1);
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

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
    uint16_t set   = gpio->pin_mask & ( gpio->new_pin_states & ~gpio->previous_pin_states);
    uint16_t clear = gpio->pin_mask & (~gpio->new_pin_states &  gpio->previous_pin_states);
    boost::unique_lock<boost::mutex> lock(CAN_mutex);
    if(gpio->servo_id == carousel_id) {
            carousel->output(set, clear);
    } else if(gpio->servo_id == port_steering_id) {
            port->steering.output(set, clear);
    } else if(gpio->servo_id == port_wheel_id) {
            port->wheel.output(set, clear);
    } else if(gpio->servo_id == starboard_steering_id) {
            starboard->steering.output(set, clear);
    } else if(gpio->servo_id == starboard_wheel_id) {
            starboard->wheel.output(set, clear);
    } else if(gpio->servo_id == stern_steering_id) {
            stern->steering.output(set, clear);
    } else if(gpio->servo_id == stern_wheel_id) {
            stern->wheel.output(set, clear);
    } else {
            ROS_ERROR("Unknown servo_id in GPIO: %d", gpio->servo_id);
    }
}

void Motion::syncCallback(CANOpen::SYNC &sync)
{
    pv_counter = 7;
    if(carousel->ready() && !gpio_enabled ) {
        carousel->outputPinFunction(1, CANOpen::Manual, std::vector<uint8_t>(), false);
        carousel->outputPinFunction(2, CANOpen::Manual, std::vector<uint8_t>(), false);
        carousel->outputPinFunction(3, CANOpen::Manual, std::vector<uint8_t>(), false);
        gpio_enabled=true;
    }
}

void Motion::reconfigureCallback(PlatformParametersConfig &config, uint32_t level)
{
    wheel_diameter = config.wheel_diameter;
    port_steering_offset = config.port_steering_offset;
    starboard_steering_offset = config.starboard_steering_offset;
    stern_steering_offset = config.stern_steering_offset;

    if( CAN_fd>0 ) {
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

void Motion::carouselCallback(const geometry_msgs::Quaternion::ConstPtr qmsg)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(*qmsg, q);
    desired_carousel_position = tf::getYaw(q); 
    carousel->setPosition(
            (desired_carousel_position - carousel_offset)*
            carousel_encoder_counts/2./M_PI);
}

}
