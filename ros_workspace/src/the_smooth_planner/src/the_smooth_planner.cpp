#include <the_smooth_planner/the_smooth_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/inflation_layer.h>
#include <platform_motion_msgs/Path.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

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
	ros::NodeHandle rootNodeHandle("/");

	localNodeHandle.param("maximum_linear_velocity", maximum_linear_velocity, 1.0);
	localNodeHandle.param("linear_acceleration", linear_acceleration, 1.0);

	this->odometry = nav_msgs::Odometry();

	pose_subscriber = parentNodeHandle.subscribe("plan", 1, &TheSmoothPlanner::setPath, this);
	odom_subscriber = parentNodeHandle.subscribe("odometry", 1, &TheSmoothPlanner::setOdometry, this);

	smooth_path_publisher = localNodeHandle.advertise<platform_motion_msgs::Path>("/motion/planned_path", 1);
}

bool TheSmoothPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
	// This is handled in the pvt_segment code instead of this local planner
	return true;
}

bool TheSmoothPlanner::isGoalReached()
{
	// The pvt_segment code in platform_motion knows how to answer this
	// question much better than the local planner.  The goal is reached
	// when all pvt segments have been executed to completion.  Since this
	// is a much better measure than some kind of tolerance-based logic here,
	// we return false in protest.
	ROS_DEBUG("IS GOAL REACHED");
	return false;
}

bool TheSmoothPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	// The global planner emits a nav_msgs::Path instead of this message type.
	// We reuse the nav_msgs::Path handler here
	if (plan.size() > 0)
	{
		nav_msgs::Path path;
		path.poses = plan;
		path.header = plan[0].header;
		this->setPath(path);
	}
	ROS_DEBUG("RECEIVED PLAN");
	return true;
}

void TheSmoothPlanner::setPath(const nav_msgs::Path& path)
{
	ROS_DEBUG("RECEIVED PATH");

	// What direction is the robot going and what is the linear/angular vel?
	Time timestamp = Time::now();
	//timestamp.sec = 0;
	//timestamp.nsec = 0;
	Eigen::Vector3d linearVelocity(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z);
	Eigen::Vector3d angularVelocity(odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z);
	double currentVelocityMagnitude = linearVelocity.norm();
	double pathVelocityMagnitude = currentVelocityMagnitude;

	platform_motion_msgs::Path path_msg;
	path_msg.knots.resize(path.poses.size());
	path_msg.header.stamp = timestamp;
	path_msg.header.seq = 0;
	path_msg.header.frame_id = "map";

	// Compute the distance we should travel before we stop accelerating
	double stopAcceleratingDistance = (maximum_linear_velocity*maximum_linear_velocity - currentVelocityMagnitude*currentVelocityMagnitude)/(2.0*linear_acceleration);
	double distanceTraveled = 0.0;

	// Populate the first entry with the current kinematic data.
	if (path.poses.size() > 0)
	{
		path_msg.knots[0].header.stamp = timestamp;
		path_msg.knots[0].header.seq = 0;
		path_msg.knots[0].header.frame_id = "map";
		path_msg.knots[0].pose = path.poses[0].pose;
		path_msg.knots[0].twist = odometry.twist.twist;
	}

	for (unsigned int i = 0; i < path.poses.size()-1; ++i)
	{
		// 1) Compute a final direction vector from the final orientation
		tf::Vector3 forwardVector(1, 0, 0);
		tf::Quaternion initialQuaternion(path.poses[i].pose.orientation.x,
                                                 path.poses[i].pose.orientation.y,
                                                 path.poses[i].pose.orientation.z,
                                                 path.poses[i].pose.orientation.w);
		tf::Quaternion finalQuaternion(path.poses[i+1].pose.orientation.x,
                                               path.poses[i+1].pose.orientation.y,
                                               path.poses[i+1].pose.orientation.z,
                                               path.poses[i+1].pose.orientation.w);
		tf::Vector3 finalLinearDirection = tf::quatRotate(finalQuaternion, forwardVector);

		// 2) Based on input V magnitude, compute the output V magnitude
		double deltaX = (path.poses[i+1].pose.position.x - path.poses[i].pose.position.x);
		double deltaY = (path.poses[i+1].pose.position.y - path.poses[i].pose.position.y);
		double linear_distance = sqrt(deltaX*deltaX + deltaY*deltaY);
		distanceTraveled += linear_distance;

		double finalLinearVelocityMagnitude = maximum_linear_velocity;
		bool isAccelerating = (distanceTraveled < stopAcceleratingDistance);
		if (isAccelerating)
		{
			finalLinearVelocityMagnitude = (maximum_linear_velocity-currentVelocityMagnitude)*(distanceTraveled/stopAcceleratingDistance) + currentVelocityMagnitude;
			finalLinearVelocityMagnitude = min(finalLinearVelocityMagnitude, maximum_linear_velocity);
		}
		tf::Vector3 finalLinearVelocity = finalLinearDirection;
		finalLinearVelocity *= finalLinearVelocityMagnitude;

		// 3) Based on input theta_dot, compute the output theta_dot
		double delta_time_seconds = 2.00*linear_distance/(finalLinearVelocityMagnitude+pathVelocityMagnitude);
		//double angle = initialQuaternion.angleShortestPath(finalQuaternion);
		tf::Quaternion initialToFinalQuaternion = initialQuaternion.inverse()*finalQuaternion;
		double angle = tf::getYaw(initialToFinalQuaternion);
		double angular_velocity = angle/delta_time_seconds;

		// 4) Store the data in a new message format containing linear and
		//    angular PVT values
		timestamp += ros::Duration(delta_time_seconds);

		path_msg.knots[i+1].header.seq = i+1;
		path_msg.knots[i+1].header.stamp = timestamp;
		path_msg.knots[i+1].header.frame_id = "map";
		path_msg.knots[i+1].pose = path.poses[i+1].pose;
		path_msg.knots[i+1].twist.linear.x = finalLinearVelocity.x();
		path_msg.knots[i+1].twist.linear.y = finalLinearVelocity.y();
		path_msg.knots[i+1].twist.linear.z = 0.0;
		path_msg.knots[i+1].twist.angular.x = 0.0;
		path_msg.knots[i+1].twist.angular.y = 0.0;
		path_msg.knots[i+1].twist.angular.z = angular_velocity;

		pathVelocityMagnitude = finalLinearVelocityMagnitude;
	}

	// 5) Handle deceleration
	double targetDecelerationVelocity = 0.0;
	for (unsigned int i = path.poses.size() - 1; i > 0; --i)
	{
		double velocitySquared = (path_msg.knots[i].twist.linear.x*path_msg.knots[i].twist.linear.x +
                                 path_msg.knots[i].twist.linear.y*path_msg.knots[i].twist.linear.y);

		if (targetDecelerationVelocity*targetDecelerationVelocity >= velocitySquared)
		{
			break;
		}

		// Update the velocity for this path point
		path_msg.knots[i].twist.linear.x *= targetDecelerationVelocity;
		path_msg.knots[i].twist.linear.y *= targetDecelerationVelocity;

		double delta_time_seconds = path_msg.knots[i].header.stamp.toSec() - path_msg.knots[i-1].header.stamp.toSec();
		targetDecelerationVelocity += linear_acceleration*delta_time_seconds;
	}

	// 6) Publish the_smooth_path and visualization messages
	smooth_path_publisher.publish(path_msg);
	return;
}

void TheSmoothPlanner::setOdometry(const nav_msgs::Odometry& odometry)
{
	ROS_DEBUG("RECEIVED ODOMETRY");
	this->odometry = odometry;
	return;
}

}
