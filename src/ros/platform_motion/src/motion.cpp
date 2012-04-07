#include <math.h>
#include <unistd.h>
#include <tr1/memory>
#include <poll.h>

#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion/HomeWheelPodsAction.h>

#include <motion/wheelpod.h>
#include <motion/motion.h>

#define POLL_TIMEOUT_MS 100

namespace platform_motion {

Motion::Motion() :
    home_action_server(nh_, "home_wheelpods", false),
    enabled(false)
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

    std::tr1::shared_ptr<CANOpen::KvaserInterface> pintf(new CANOpen::KvaserInterface(
                                                CAN_channel,
                                                CAN_baud
                                               ));

    CAN_fd = pintf->getFD();

    pbus = std::tr1::shared_ptr<CANOpen::Bus>(new CANOpen::Bus(pintf, false));

    carousel = std::tr1::shared_ptr<CANOpen::CopleyServo>(new
            CANOpen::CopleyServo(carousel_id, sync_interval, pbus));

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

    twist_sub = nh_.subscribe("twist", 2, &Motion::twistCallback, this);

    home_action_server.registerGoalCallback(boost::bind(&Motion::doHome, this));

}

Motion::~Motion()
{
    CAN_thread_run = false;
    CAN_thread.join();
    close(notify_read_fd);
    close(notify_write_fd);
}

void Motion::start(void)
{
    int pipe_fds[2];
    if(pipe(pipe_fds)<0) {
        perror("Error creating pipe fds");
    }
    notify_read_fd = pipe_fds[0];
    notify_write_fd = pipe_fds[1];
    ROS_INFO("Start CAN bus thread");
    CAN_thread = boost::thread(boost::bind(&Motion::runBus, this));
    ROS_INFO("Start home server");
    home_action_server.start();
    ROS_INFO("Send initialize notification");
    char c='\x03';
    if(write(notify_write_fd, &c, 1) != 1) {
        perror("Error during start notify write");
    }
}

/*
void Motion::shutdown(void)
{
    port->enable(false);
    stern->enable(false);
    starboard->enable(false);
    carousel->enable(false);
}
*/

void Motion::runBus(void)
{
    struct pollfd pfd[2];
    pfd[0].fd = CAN_fd;
    pfd[0].events = POLLIN | POLLOUT;
    pfd[1].fd = notify_read_fd;
    pfd[1].events = POLLIN;
    int ret=0;
    CAN_thread_run=true;
    while(ret >= 0 && CAN_thread_run ){
        ret = poll(pfd, 2, POLL_TIMEOUT_MS);
        if(ret>0) {
            if(pfd[0].revents) {
                bool canRead = pfd[0].revents&POLLIN;
                bool canWrite = pfd[0].revents&POLLOUT;
                pbus->runonce(canRead, canWrite);
                pfd[0].events |= POLLIN;
                if(canWrite) {
                    pfd[0].events |= POLLOUT;
                } else {
                    pfd[0].events &= ~POLLOUT;
                }
            } else if(pfd[1].revents) {
                char c;
                if(read(notify_read_fd, &c, 1) == 1){
                    CANOpen::DS301CallbackObject
                        cb(static_cast<CANOpen::TransferCallbackReceiver *>(this),
                                static_cast<CANOpen::DS301CallbackObject::CallbackFunction>(&Motion::homeComplete));
                    switch(c) {
                        case '\x00':
                            port->drive(angle_port, v_port);
                            starboard->drive(angle_starboard, -v_starboard);
                            stern->drive(angle_stern, -v_stern);
                            break;
                        case '\x01':
                            std::cout << "Home" << std::endl;
                            port->home(cb);
                            starboard->home(cb);
                            stern->home(cb);
                            break;
                        case '\x02':
                            std::cout << "enable" << std::endl;
                            port->enable();
                            starboard->enable();
                            stern->enable();
                            break;
                        case '\x03':
                            std::cout << "initialize" << std::endl;
                            port->initialize();
                            starboard->initialize();
                            stern->initialize();
                            carousel->initialize();
                            break;
                        default:
                            break;
                    }
                } else {
                    perror("Error during notify read");
                }
                if(pbus->canSend()) {
                    pfd[0].events |= POLLOUT;
                }
            }
        }
        if(ready() && ! enabled) {
            enable();
        }
    }
}
 
void Motion::twistCallback(const geometry_msgs::Twist::ConstPtr twist)
{
    Eigen::Vector2d body_pt;
    body_pt << center_pt_x, center_pt_y;
    Eigen::Vector2d body_vel;
    body_vel << twist->linear.x, twist->linear.y;
    double body_omega = twist->angular.z;
    double kappa;
    Eigen::Vector2d center_pos;
    double body_speed = body_vel.norm();
    double phi;
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
        Eigen::Vector2d starboard_pos;
        starboard_pos << length, width/2;
        v_port =
            (center_pos-starboard_pos).norm()*body_omega/M_PI/wheel_diameter;
        Eigen::Vector2d port_pos;
        port_pos << length, -width/2;
        v_starboard =
            (center_pos-port_pos).norm()*body_omega/M_PI/wheel_diameter;
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
    char c='\x00';
    if(write(notify_write_fd, &c, 1) != 1) {
        perror("Error during move notify write");
    }

}

void Motion::doHome(void)
{
    home_count = 0;
    char c='\x01';
    if(write(notify_write_fd, &c, 1) != 1) {
        perror("Error during home notify write");
    }
    home_action_server.acceptNewGoal();
}

void Motion::homeComplete(CANOpen::DS301 &node)
{
    home_count++;
    feedback.home_count = home_count;
    home_action_server.publishFeedback(feedback);
    if(home_count == 3) {
        home_action_server.setSucceeded(result);  
    }
}

bool Motion::ready(void)
{
    return port->ready() && starboard->ready() && stern->ready() && carousel->ready();
}

void Motion::enable(void)
{
    char c='\x02';
    if(write(notify_write_fd, &c, 1) != 1) {
        perror("Error during enable notify write");
    }
    enabled = true;
}

}
