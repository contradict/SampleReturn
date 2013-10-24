#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <actionlib/client/simple_action_client.h>
#include <platform_motion/HomeAction.h>
#include <platform_motion/Enable.h>
#include <platform_motion/SelectCommandSource.h>
#include <manipulator/ManipulatorAction.h>
#include <visual_servo/VisualServoAction.h>

class Teleop
{
public:
  Teleop();

private:
  double scale_joystick(double scale, double exponent, double value);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void doHoming(void);
  void doGrab(void);
  void doServo(void);
  void timerCallback(const ros::TimerEvent &);
  
  ros::NodeHandle nh_;

  ros::Timer pub_timer;

  int linear_x, linear_y, angular_z;
  int button_homing, button_grab, button_servo, button_cancel_servo;
  double l_scale_, l_exp_;
  double a_scale_, a_exp_;
  ros::Publisher twist_pub_;
  ros::Subscriber joy_sub_;
  bool homing, homed;
  actionlib::SimpleActionClient<platform_motion::HomeAction> home_pods;
  actionlib::SimpleActionClient<manipulator::ManipulatorAction> manipulate;
  actionlib::SimpleActionClient<visual_servo::VisualServoAction> servo;
  bool servo_active;

  double vel_x, vel_y, vel_theta;
};


Teleop::Teleop():
  linear_x(1),
  linear_y(0),
  angular_z(2),
  button_homing(9),
  button_grab(3),
  button_servo(1),
  button_cancel_servo(2),
  l_scale_(2),
  l_exp_(2),
  a_scale_(M_PI),
  a_exp_(2),
  homing(false),
  homed(false),
  home_pods("wheel_pods/home"),
  manipulate("manipulator/grab_action"),
  servo("visual_servo_action"),
  servo_active(false),
  vel_x(0),
  vel_y(0),
  vel_theta(0)
{
    nh_.param("/teleop/axis_linear_x", linear_x, linear_x);
    nh_.param("/teleop/axis_linear_y", linear_y, linear_y);
    nh_.param("/teleop/axis_angular_z", angular_z, angular_z);
    nh_.param("/teleop/button_homing", button_homing, button_homing);
    nh_.param("/teleop/scale_angular", a_scale_, a_scale_);
    nh_.param("/teleop/exponent_angular", a_exp_, a_exp_);
    nh_.param("/teleop/scale_linear", l_scale_, l_scale_);
    nh_.param("/teleop/exponent_linear", l_exp_, l_exp_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);

    pub_timer = nh_.createTimer(ros::Duration(0.050), &Teleop::timerCallback, this);

}

void Teleop::doHoming(void)
{
    homing=true;
    homed=false;
    platform_motion::SelectCommandSource sel;
    sel.request.source="test";
    ros::service::call("CAN_select_command_source", sel);
    std::string savedsource(sel.response.source);
    sel.request.source="None";
    ros::service::call("CAN_select_command_source", sel);
    sel.request.source=savedsource;
    ros::Duration(0.25).sleep();
    ROS_INFO("Waiting for homing server");
    if(!home_pods.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Timeout waiting for homing server");
        homing=false;
        ros::service::call("CAN_select_command_source", sel);
        return;
    }
    ROS_INFO("Send home goal");
    platform_motion::HomeGoal g;
    g.home_count = 3;
    home_pods.sendGoal(g);
    if(!home_pods.waitForResult(ros::Duration(60.0))) {
        ROS_ERROR("Timeout waiting for homing");
        home_pods.cancelGoal();
    } else {
        homed=true;
        ROS_INFO("Homing complete");
    }
    homing=false;
    ros::service::call("CAN_select_command_source", sel);
}

void Teleop::doGrab(void)
{
    ROS_INFO("Waiting for manipulator server");
    if(!manipulate.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Timeout waiting for manipulator server");
        return;
    }
    ROS_INFO("Send manipulator goal");
    manipulator::ManipulatorGoal grab;
    grab.type = grab.GRAB;
    grab.grip_torque=0.7;
    grab.target_bin=2;
    manipulate.sendGoal(grab);
    if(!manipulate.waitForResult(ros::Duration(60.0))) {
        manipulate.cancelGoal();
        ROS_ERROR("Timeout waiting for grab");
    } else {
        ROS_INFO("Grab complete");
    }
}

void Teleop::doServo(void)
{
    platform_motion::SelectCommandSource sel;
    sel.request.source="test";
    ros::service::call("CAN_select_command_source", sel);
    std::string savedsource(sel.response.source);
    sel.request.source="Servo";
    ros::service::call("CAN_select_command_source", sel);
    sel.request.source=savedsource;
    ROS_INFO("Waiting for servo server");
    if(!manipulate.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Timeout waiting for servo server");
        ros::service::call("CAN_select_command_source", sel);
        return;
    }
    ROS_INFO("Send servo goal");
    visual_servo::VisualServoGoal s;
    s.do_align_object_to_manipulator = true;
    servo.sendGoal(s);
    servo_active = true;
    while(true)
    {
        if(servo.waitForResult(ros::Duration(0.1)))
        {
            ROS_INFO("Servo complete");
            servo_active = false;
            break;
        }
        if(!servo_active) {
            servo.cancelGoal();
            ROS_INFO("Servo canceled");
            break;
        }
    }
    ros::service::call("CAN_select_command_source", sel);
}


double Teleop::scale_joystick(double scale, double exponent, double value)
{
    return copysign(scale*pow(fabs(value), exponent), value);
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[button_homing] && !homing)
    {
        boost::thread(boost::bind(&Teleop::doHoming, this));
    }
    if(joy->buttons[button_grab])
    {
        boost::thread(boost::bind(&Teleop::doGrab, this));
    }
    if(joy->buttons[button_servo] && !servo_active)
    {
        boost::thread(boost::bind(&Teleop::doServo, this));
    }
    if(joy->buttons[button_cancel_servo] && servo_active)
    {
        servo_active=false;
    }
    vel_theta=scale_joystick(a_scale_, a_exp_, joy->axes[angular_z]);
    vel_x=scale_joystick(l_scale_, l_exp_, joy->axes[linear_x]);
    vel_y=scale_joystick(l_scale_, l_exp_, joy->axes[linear_y]);
}

void Teleop::timerCallback(const ros::TimerEvent &evt)
{
    if( !homing )
    {
        geometry_msgs::Twist twist;
        twist.angular.z = vel_theta;
        twist.linear.x = vel_x;
        twist.linear.y = vel_y;
        twist_pub_.publish(twist);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop teleop;

  ros::spin();
}
