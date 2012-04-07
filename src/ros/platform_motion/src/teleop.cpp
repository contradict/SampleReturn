#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <actionlib/client/simple_action_client.h>
#include <platform_motion/HomeWheelPodsAction.h>


class Teleop
{
public:
  Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void doHoming(void);
  
  ros::NodeHandle nh_;

  int linear_x, linear_y, angular_z, button_homing;
  double l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  ros::Subscriber joy_sub_;
  bool homing;
  actionlib::SimpleActionClient<platform_motion::HomeWheelPodsAction> ac;
  
};


Teleop::Teleop():
  linear_x(1),
  linear_y(0),
  angular_z(2),
  button_homing(1),
  homing(false),
  ac("home_wheelpods")
{
  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("axis_angular_z", angular_z, angular_z);
  nh_.param("button_homing", button_homing, button_homing);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);

}

void Teleop::doHoming(void)
{
    homing=true;
    ROS_INFO("Waiting for homing server");
    if(!ac.waitForServer(ros::Duration(30.0))) {
        ROS_ERROR("Timeout waiting for homing server");
        homing=false;
        return;
    }
    ROS_INFO("Send home goal");
    platform_motion::HomeWheelPodsGoal g;
    ac.sendGoal(g);
    if(!ac.waitForResult(ros::Duration(60.0))) {
        ROS_ERROR("Timeout waiting for homing");
    } else {
        ROS_INFO("Homing complete");
    }
    homing=false;
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(homing) {
        // Cancel?
    } else {
        if(joy->buttons[button_homing]) {
            boost::thread(boost::bind(&Teleop::doHoming, this));
        } else {
            geometry_msgs::Twist twist;
            twist.angular.z = a_scale_*joy->axes[angular_z];
            twist.linear.x = l_scale_*joy->axes[linear_x];
            twist.linear.y = l_scale_*joy->axes[linear_y];
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
