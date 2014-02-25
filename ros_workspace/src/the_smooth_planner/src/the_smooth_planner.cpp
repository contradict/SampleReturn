#include <the_smooth_planner/the_smooth_planner.h>
#include <pluginlib/class_list_macros.h>
#include <the_smooth_planner/TheSmoothPlannerStats.h>

#include <costmap_2d/inflation_layer.h>

using namespace std;
using namespace ros;


PLUGINLIB_DECLARE_CLASS(the_smooth_planner, TheSmoothPlanner, the_smooth_planner::TheSmoothPlanner, nav_core::BaseLocalPlanner);

namespace the_smooth_planner
{

TheSmoothPlanner::TheSmoothPlanner()
{
}

TheSmoothPlanner::TheSmoothPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name, tf, costmap_ros);
}

    
void TheSmoothPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
	ros::NodeHandle localNodeHandle("~/" + name);
	ros::NodeHandle parentNodeHandle("~/");

	localNodeHandle.param("maximum_velocity", maximum_velocity, 1.0);
	localNodeHandle.param("acceleration", acceleration, 1.0);

	pose_subscriber = parentNodeHandle.subscribe("plan", 1, &TheSmoothPlanner::setPath, this);

	pose_publisher = localNodeHandle.advertise<nav_msgs::Path>("smooth_plan", 1);
}

bool TheSmoothPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("COMPUTE VELOCITY COMMANDS");
	return false;
}

bool TheSmoothPlanner::isGoalReached()
{
	// TODO - return true when the goal pose specified by the global planner
	//        has been reached
	ROS_INFO("IS GOAL REACHED");
	return false;
}

bool TheSmoothPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	// TODO - Set the local planner from the input global plan.  Make sure
	//        the local plan fits inside the grid
	ROS_INFO("RECEIVED PLAN");
	return true;
}

void TheSmoothPlanner::setPath(const nav_msgs::Path& path)
{
	pose_publisher.publish(path);
	return;
}

}
