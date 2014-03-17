#include <the_smooth_planner/the_smooth_planner.h>
#include <pluginlib/class_list_macros.h>
#include <the_smooth_planner/TheSmoothPlannerStats.h>
#include <costmap_2d/inflation_layer.h>
#include <ecl/geometry/cubic_spline.hpp>

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
	localNodeHandle.param("maximum_angular_velocity", maximum_angular_velocity, 1.0);
	localNodeHandle.param("linear_acceleration", linear_acceleration, 1.0);
	localNodeHandle.param("angular_acceleration", angular_acceleration, 1.0);

	pose_subscriber = parentNodeHandle.subscribe("SBPLLatticePlanner/plan", 1, &TheSmoothPlanner::setPath, this);
	odom_subscriber = parentNodeHandle.subscribe("odometry", 1, &TheSmoothPlanner::setOdometry, this);

	pose_publisher = localNodeHandle.advertise<nav_msgs::Path>("smooth_plan", 1);
}

bool TheSmoothPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("COMPUTE VELOCITY COMMANDS");
	return true;
}

bool TheSmoothPlanner::isGoalReached()
{
	// TODO - return true when the goal pose specified by the global planner
	//        has been reached
	ROS_INFO("IS GOAL REACHED");
	return true;
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
	ROS_INFO("RECEIVED PATH");

	// What direction is the robot going and what is the linear/angular vel?
	time timestamp = now();
	Eigen::Vector3d linearVelocity(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z);
	double linearVelocityMagnitude = linearVelocity.norm();

	// TODO - push the current odometry information into the output
	//        the_smooth_path message as the first entry

	for (unsigned int i = 0; i < path.poses.size()-1; ++i)
	{
		// 1) Fit a cubic spline to each pair of points, given P and V direction
		Array<double, 2> times;
		Array<double, 2> positions_x;
		Array<double, 2> positions_y;
		Array<double, 2> angles;
		double linear_velocity_xf;
		double linear_velocity_yf;
		times << 0.00, 1.00;
		positions_x << path.poses[i].pose.position.x, path.poses[i+1].pose.position.x;
		positions_y << path.poses[i].pose.position.y, path.poses[i+1].pose.position.y;
		btQuaternion initialOrientationQuat(path.poses[i].pose.orientation.x,
                                            path.poses[i].pose.orientation.y,
                                            path.poses[i].pose.orientation.z,
                                            path.poses[i].pose.orientation.w);
		btQuaternion finalOrientationQuat(path.poses[i+1].pose.orientation.x,
                                          path.poses[i+1].pose.orientation.y,
                                          path.poses[i+1].pose.orientation.z,
                                          path.poses[i+1].pose.orientation.w);
		double roll_0, pitch_0, yaw_0;
		double roll_f, pitch_f, yaw_f;
		tf::Matrix3x3(initialOrientationQuat).getRPY(roll_0, pitch_0, yaw_0);
		tf::Matrix3x3(finalOrientationQuat).getRPY(roll_f, pitch_f, yaw_f);
		angles << yaw_0, yaw_f;
		linear_velocity_xf = cos(yaw_f);
		linear_velocity_yf = sin(yaw_f);
		CubicSpline positionXSpline = CubicSpline::ContinuousDerivatives(times, positions_x, linearVelocity.x, linear_velocity_xf);
		CubicSpline positionYSpline = CubicSpline::ContinuousDerivatives(times, positions_y, linearVelocity.y, linear_velocity_yf);

		// 2) Based on input V magnitude, compute the output V magnitude
		// 3) Based on input theta_dot, compute the output theta_dot
		double linear_distance = 0; // Compute the spline length
		double finalLinearVelocityMagnitude = linearVelocityMagnitude;
		bool isAccelerating = linearVelocityMagnitude < maximum_linear_velocity;
		if (isAccelerating)
		{
			finalLinearVelocityMagnitude = sqrt(linearVelocity.normSquared() + 2*linear_acceleration*linear_distance);
			finalLinearVelocityMagnitude = min(finalLinearVelocityMagnitude, maximum_linear_velocity);
		}

		// 4) Store the data in a new message format containing PVT for linear
		//    and angular metrics
		Eigen::Vector2d finalLinearVelocity(linear_velocity_xf, linear_velocity_yf);
		finalLinearVelocity.normalize();
		finalLinearVelocity *= finalLinearVelocityMagnitude;

		double delta_time_seconds = 2.00*linear_distance/(finalLinearVelocityMagnitude+linearVelocityMagnitude);
		timestamp.sec += static_cast<int>(delta_time_seconds);
		timestamp.nsec += static_cast<int>((delta_time_seconds - static_cast<int>(delta_time_seconds))*1000000000);

		the_smooth_msgs::the_smooth_path the_smooth_path;
		the_smooth_path.header.stamp = timestamp;
		the_smooth_path.pose = poses[i+1];
		the_smooth_path.twist.linear.x = finalLinearVelocity.x;
		the_smooth_path.twist.linear.y = finalLinearVelocity.y;
		the_smooth_path.twist.linear.z = finalLinearVelocity.z;
		the_smooth_path.twist.angular.x = 0.0;
		the_smooth_path.twist.angular.y = 0.0;
		the_smooth_path.twist.angular.z = 0.0;
	}

	// 5) Publish the_smooth_path message
	this->path = path;
	pose_publisher.publish(path);
	return;
}

void TheSmoothPlanner::setOdometry(const nav_msgs::Odometry& odometry)
{
	ROS_INFO("RECEIVED ODOMETRY");
	this->odometry = odometry;
	return;
}

}
