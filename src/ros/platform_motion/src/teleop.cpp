#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <actionlib/client/simple_action_client.h>
#include <platform_motion/HomeWheelPodsAction.h>
#include <platform_motion/EnableWheelPodsAction.h>


class Teleop
{
public:
  Teleop();

private:
  double scale_joystick(double scale, double exponent, double value);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void doHoming(void);
  void doEnable(bool state);
  
  ros::NodeHandle nh_;

  int linear_x, linear_y, angular_z;
  int button_homing, button_enable, button_disable;
  double l_scale_, l_exp_;
  double a_scale_, a_exp_;
  ros::Publisher twist_pub_;
  ros::Subscriber joy_sub_;
  bool homing, homed;
  bool enabling, enabled;
  actionlib::SimpleActionClient<platform_motion::HomeWheelPodsAction> hac;
  actionlib::SimpleActionClient<platform_motion::EnableWheelPodsAction> eac;
  
};


Teleop::Teleop():
  linear_x(1),
  linear_y(0),
  angular_z(2),
  button_homing(1),
  button_enable(2),
  button_disable(3),
  l_scale_(2),
  l_exp_(2),
  a_scale_(M_PI),
  a_exp_(2),
  homing(false),
  homed(false),
  enabling(false),
  enabled(false),
  hac("home_wheelpods"),
  eac("enable_wheelpods")
{
    nh_.param("/teleop/axis_linear_x", linear_x, linear_x);
    nh_.param("/teleop/axis_linear_y", linear_y, linear_y);
    nh_.param("/teleop/axis_angular_z", angular_z, angular_z);
    nh_.param("/teleop/button_homing", button_homing, button_homing);
    nh_.param("/teleop/button_enable", button_enable, button_enable);
    nh_.param("/teleop/button_disable", button_disable, button_disable);
    nh_.param("/teleop/scale_angular", a_scale_, a_scale_);
    nh_.param("/teleop/exponent_angular", a_exp_, a_exp_);
    nh_.param("/teleop/scale_linear", l_scale_, l_scale_);
    nh_.param("/teleop/exponent_linear", l_exp_, l_exp_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);

}

void Teleop::doHoming(void)
{
    homing=true;
    homed=false;
    ROS_INFO("Waiting for homing server");
    if(!hac.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Timeout waiting for homing server");
        homing=false;
        return;
    }
    ROS_INFO("Send home goal");
    platform_motion::HomeWheelPodsGoal g;
    hac.sendGoal(g);
    if(!hac.waitForResult(ros::Duration(60.0))) {
        ROS_ERROR("Timeout waiting for homing");
    } else {
        homed=true;
        ROS_INFO("Homing complete");
    }
    homing=false;
}

void Teleop::doEnable(bool state)
{
    enabling=true;
    ROS_INFO("Waiting for enable server");
    if(!eac.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Timeout waiting for enable server");
        enabling=false;
        return;
    }
    ROS_INFO("Send enable goal: %s", state?"true":"false");
    platform_motion::EnableWheelPodsGoal g;
    g.enable_state = state;
    eac.sendGoal(g);
    if(!eac.waitForResult(ros::Duration(5.0))) {
        ROS_ERROR("Timeout waiting for enable");
    } else {
        ROS_INFO("enable complete");
        enabled = eac.getResult()->stern_enabled;
    }
    enabling=false;
}


double Teleop::scale_joystick(double scale, double exponent, double value)
{
    return copysign(scale*pow(fabs(value), exponent), value);
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(homing) {
        // Cancel?
    } else {
        if(joy->buttons[button_homing]) {
            boost::thread(boost::bind(&Teleop::doHoming, this));
        } else if(joy->buttons[button_enable]) {
            boost::thread(boost::bind(&Teleop::doEnable, this, _1), true);
        } else if(joy->buttons[button_disable]) {
            boost::thread(boost::bind(&Teleop::doEnable, this, _1), false);
        } else {
            geometry_msgs::Twist twist;
            twist.angular.z = scale_joystick(a_scale_, a_exp_, joy->axes[angular_z]);
            twist.linear.x = scale_joystick(l_scale_, l_exp_, joy->axes[linear_x]);
            twist.linear.y = scale_joystick(l_scale_, l_exp_, joy->axes[linear_y]);
            twist_pub_.publish(twist);
        }
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop teleop;

  ros::spin();
}
