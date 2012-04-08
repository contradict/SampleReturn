#include <math.h>
#include <unistd.h>
#include <tr1/memory>
#include <poll.h>
#include <algorithm>

#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <canlib.h>
#include <CANOpen.h>
#include <KvaserInterface.h>

#include <actionlib/server/simple_action_server.h>
#include <platform_motion/HomeWheelPodsAction.h>

#include <motion/wheelpod.h>
#include <motion/motion.h>

#include <motion/lmmin.h>

#define POLL_TIMEOUT_MS 100

namespace platform_motion {

struct lmmin_data {
    Eigen::Vector2d body_pt;
    Eigen::Vector2d port_pos, port_move;
    Eigen::Vector2d stern_pos, stern_move;
    Eigen::Vector2d starboard_pos, starboard_move;
};

void lmmin_evaluate(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info)
{
    const struct lmmin_data *data = reinterpret_cast<const struct lmmin_data *>(vdata);

    Eigen::Matrix2d R;
    R(0,0) = cos(xytheta[2]);
    R(0,1) = -sin(xytheta[2]);
    R(1,0) = sin(xytheta[2]);
    R(1,1) = cos(xytheta[2]);

    Eigen::Vector2d T(xytheta[0], xytheta[1]);
    T += data->body_pt;

    Eigen::Vector2d port_error = R*(data->port_pos - data->body_pt) + T - data->port_move;
    Eigen::Vector2d stern_error = R*(data->stern_pos - data->body_pt) + T - data->stern_move;
    Eigen::Vector2d starboard_error = R*(data->starboard_pos - data->body_pt) + T - data->starboard_move;

    fvec[0] = port_error(0);
    fvec[1] = port_error(1);
    fvec[2] = stern_error(0);
    fvec[3] = stern_error(1);
    fvec[4] = starboard_error(0);
    fvec[5] = starboard_error(1);
    *info = 1;
}

Motion::Motion() :
    home_action_server(nh_, "home_wheelpods", false),
    enabled(false),
    last_starboard_wheel(0), last_port_wheel(0), last_stern_wheel(0),
    pv_counter(0),
    odom_position(Eigen::Vector2d::Zero()),
    odom_orientation(0),
    odom_frame_id(0)
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

    port_pos << length, width/2;
    starboard_pos << length, -width/2;
    stern_pos << 0, 0;
    body_pt << center_pt_x, center_pt_y;

    std::tr1::shared_ptr<CANOpen::KvaserInterface> pintf(new CANOpen::KvaserInterface(
                                                CAN_channel,
                                                CAN_baud
                                               ));

    CAN_fd = pintf->getFD();

    pbus = std::tr1::shared_ptr<CANOpen::Bus>(new CANOpen::Bus(pintf, false));

    std::tr1::shared_ptr<CANOpen::SYNC> psync(new CANOpen::SYNC(0,
                CANOpen::SYNCCallbackObject(
                static_cast<CANOpen::TransferCallbackReceiver *>(this),
                static_cast<CANOpen::SYNCCallbackObject::CallbackFunction>(&Motion::syncCallback))));

    pbus->add(psync);

    carousel = std::tr1::shared_ptr<CANOpen::CopleyServo>(new
            CANOpen::CopleyServo(carousel_id, sync_interval, pbus));

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
    port->pvCallbacks(pvcb, pvcb);

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
    starboard->pvCallbacks(pvcb, pvcb);

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
    stern->pvCallbacks(pvcb, pvcb);

    twist_sub = nh_.subscribe("twist", 2, &Motion::twistCallback, this);

    odometry_pub = nh_.advertise<nav_msgs::Odometry>("odometry", 1);

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
        bool rdy = ready();
        if(rdy && ! enabled) {
            enable();
        } else if( !rdy && enabled ) {
            enabled = false;
        }
    }
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

void Motion::pvCallback(CANOpen::DS301 &node)
{
    if(pv_counter>0 && --pv_counter==0) {
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
        /*
        std::cerr << "vel: (" << port_wheel_velocity << ", "
            << starboard_wheel_velocity << ", " << stern_wheel_velocity << ") "
            << max_abs_vel << std::endl;
            */
        nav_msgs::Odometry odo;
        odo.header.frame_id = "odom";
        odo.header.stamp = ros::Time::now();
        if(max_abs_vel<1e-2) {
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
        } else {
            struct lmmin_data data;
            data.body_pt = body_pt;
            data.port_pos = port_pos;
            data.port_move = port_pos + port_delta*port_wheel_direction;
            data.stern_pos = stern_pos;
            data.stern_move = stern_pos + stern_delta*stern_wheel_direction;
            data.starboard_pos = starboard_pos;
            data.starboard_move = starboard_pos + starboard_delta*starboard_wheel_direction;

            double xytheta[3] = {0.0, 0.0, 0.0};
            lm_status_struct status;
            lm_control_struct control = lm_control_double;
            control.printflags = 0;
            lmmin( 3, xytheta, 6, &data, lmmin_evaluate, &control, &status, NULL);

            if(status.info>4) {
                // minimization failed
                std::cerr << "Minimization failed: " << lm_infmsg[status.info] << std::endl;
            } else {
                if(++odom_frame_id == 20) {
                    odom_frame_id = 0;
                    //std::cerr << "Minimization success: (" << xytheta[0] << ", "
                    //    << xytheta[1] << ", " << xytheta[2] << ")" << std::endl;
                    //std::cerr << "delta: (" << port_delta << ", "
                    //    << starboard_delta << ", " << stern_delta << ")" << std::endl;
                }
                Eigen::Matrix2d R;
                R << cos(odom_orientation), -sin(odom_orientation),
                  sin(odom_orientation),  cos(odom_orientation);
                odom_position += R*Eigen::Vector2d(xytheta[0], xytheta[1]);
                odom_orientation += xytheta[2];
                if(odom_orientation > 2*M_PI) odom_orientation -= 2*M_PI;
                if(odom_orientation < 0) odom_orientation += 2*M_PI;
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
        }
        odo.pose.pose.position.x = odom_position(0);
        odo.pose.pose.position.y = odom_position(1);
        odo.pose.pose.position.z = 0;
        odo.pose.pose.orientation.x = 0;
        odo.pose.pose.orientation.y = 0;
        odo.pose.pose.orientation.z = sin(odom_orientation/2.);
        odo.pose.pose.orientation.w = cos(odom_orientation/2.);
        odometry_pub.publish(odo);
    }
}

void Motion::syncCallback(CANOpen::SYNC &sync)
{
    pv_counter = 6;
}

}
