#include <the_smooth_planner/the_smooth_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/inflation_layer.h>
#include <ecl/geometry/cubic_spline.hpp>
#include <the_smooth_planner/TheSmoothPlannerMsg.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace ros;
using namespace ecl;

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

	this->odometry = nav_msgs::Odometry();

	pose_subscriber = parentNodeHandle.subscribe("SBPLLatticePlanner/plan", 1, &TheSmoothPlanner::setPath, this);
	odom_subscriber = parentNodeHandle.subscribe("odometry", 1, &TheSmoothPlanner::setOdometry, this);

	smooth_path_publisher = localNodeHandle.advertise<the_smooth_planner::TheSmoothPlannerMsg>("smooth_path", 1);
	visualization_publisher = localNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
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
	Time timestamp = Time::now();
	Eigen::Vector3d linearVelocity(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z);
	double linearVelocityMagnitude = linearVelocity.norm();

	the_smooth_planner::TheSmoothPlannerMsg the_smooth_planner_msg;
	the_smooth_planner_msg.poses.resize(path.poses.size());
	the_smooth_planner_msg.twists.resize(path.poses.size());
	the_smooth_planner_msg.header.stamp = timestamp;
	the_smooth_planner_msg.header.seq = 0;

	visualization_msgs::MarkerArray visualizationMarkerArray;
	visualizationMarkerArray.markers.resize(path.poses.size()+1);
	visualizationMarkerArray.markers[0].points.resize(path.poses.size());

	if (path.poses.size() > 0)
	{
		// Populate the first entry with the current kinematic data.
		the_smooth_planner_msg.poses[0].header.stamp = timestamp;
		the_smooth_planner_msg.poses[0].header.seq = 0;
		the_smooth_planner_msg.poses[0].pose = path.poses[0].pose;
		the_smooth_planner_msg.twists[0] = odometry.twist.twist;

		visualizationMarkerArray.markers[0].header.seq = 0;
		visualizationMarkerArray.markers[0].header.frame_id = "map";
		visualizationMarkerArray.markers[0].header.stamp = timestamp;
		visualizationMarkerArray.markers[0].ns = "SmoothPlannerVisualization";
		visualizationMarkerArray.markers[0].id = 0;
		visualizationMarkerArray.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
		visualizationMarkerArray.markers[0].action = visualization_msgs::Marker::ADD;
		visualizationMarkerArray.markers[0].scale.x = 0.1;
		visualizationMarkerArray.markers[0].color.r = 0.0;
		visualizationMarkerArray.markers[0].color.g = 1.0;
		visualizationMarkerArray.markers[0].color.b = 0.0;
		visualizationMarkerArray.markers[0].color.a = 1.0;
		visualizationMarkerArray.markers[0].points[0] = path.poses[0].pose.position;
	}

	for (unsigned int i = 0; i < path.poses.size()-1; ++i)
	{
		// 1) Fit a cubic spline to each pair of points, given P and V direction
		Array<double> times(2);
		Array<double> positions_x(2);
		Array<double> positions_y(2);
		Array<double> angles(2);
		double linear_velocity_xf;
		double linear_velocity_yf;

		times << 0.00, 1.00;
		positions_x << path.poses[i].pose.position.x, path.poses[i+1].pose.position.x;
		positions_y << path.poses[i].pose.position.y, path.poses[i+1].pose.position.y;
		tf::Quaternion initialOrientationQuat(path.poses[i].pose.orientation.x,
                                            path.poses[i].pose.orientation.y,
                                            path.poses[i].pose.orientation.z,
                                            path.poses[i].pose.orientation.w);
		tf::Quaternion finalOrientationQuat(path.poses[i+1].pose.orientation.x,
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
		CubicSpline positionXSpline = CubicSpline::ContinuousDerivatives(times, positions_x, linearVelocity(0), linear_velocity_xf);
		CubicSpline positionYSpline = CubicSpline::ContinuousDerivatives(times, positions_y, linearVelocity(1), linear_velocity_yf);

		// 2) Based on input V magnitude, compute the output V magnitude
		// 3) Based on input theta_dot, compute the output theta_dot
		double linear_distance = 0; // Compute the spline length
		double finalLinearVelocityMagnitude = linearVelocityMagnitude;
		bool isAccelerating = linearVelocityMagnitude < maximum_linear_velocity;
		if (isAccelerating)
		{
			finalLinearVelocityMagnitude = sqrt(linearVelocity.squaredNorm() + 2*linear_acceleration*linear_distance);
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

		the_smooth_planner_msg.poses[i+1].header.seq = i+1;
		the_smooth_planner_msg.poses[i+1].header.stamp = timestamp;
		the_smooth_planner_msg.poses[i+1].pose = path.poses[i+1].pose;
		the_smooth_planner_msg.twists[i+1].linear.x = finalLinearVelocity(0);
		the_smooth_planner_msg.twists[i+1].linear.y = finalLinearVelocity(1);
		the_smooth_planner_msg.twists[i+1].linear.z = 0.0;
		the_smooth_planner_msg.twists[i+1].angular.x = 0.0;
		the_smooth_planner_msg.twists[i+1].angular.y = 0.0;
		the_smooth_planner_msg.twists[i+1].angular.z = 0.0;

		// Store the position for the line strip
		visualizationMarkerArray.markers[0].points[i+1] = path.poses[i+1].pose.position;

		// Store the velocity at this path point
		visualizationMarkerArray.markers[i+1].header.seq = i+1;
		visualizationMarkerArray.markers[i+1].header.frame_id = "map";
		visualizationMarkerArray.markers[i+1].header.stamp = timestamp;
		visualizationMarkerArray.markers[i+1].ns = "SmoothPlannerVisualization";
		visualizationMarkerArray.markers[i+1].id = i+1;
		visualizationMarkerArray.markers[i+1].type = visualization_msgs::Marker::ARROW;
		visualizationMarkerArray.markers[i+1].action = visualization_msgs::Marker::ADD;
		visualizationMarkerArray.markers[i+1].scale.x = 0.1;
		visualizationMarkerArray.markers[i+1].scale.y = 0.1;
		visualizationMarkerArray.markers[i+1].scale.z = finalLinearVelocityMagnitude;
		visualizationMarkerArray.markers[i+1].color.r = 1.0;
		visualizationMarkerArray.markers[i+1].color.g = 0.0;
		visualizationMarkerArray.markers[i+1].color.b = 0.0;
		visualizationMarkerArray.markers[i+1].color.a = 1.0;
		visualizationMarkerArray.markers[i+1].pose.position = path.poses[i+1].pose.position;
		visualizationMarkerArray.markers[i+1].pose.orientation = path.poses[i+1].pose.orientation;
		//visualizationMarkerArray.markers[i+1].pose.orientation.x = 0.0;
		//visualizationMarkerArray.markers[i+1].pose.orientation.y = 0.0;
		//visualizationMarkerArray.markers[i+1].pose.orientation.z = 1.0;
		//visualizationMarkerArray.markers[i+1].pose.orientation.w = 0.0;
	}

	// 5) Publish the_smooth_path and visualization messages
	smooth_path_publisher.publish(the_smooth_planner_msg);
	visualization_publisher.publish(visualizationMarkerArray);
	return;
}

void TheSmoothPlanner::setOdometry(const nav_msgs::Odometry& odometry)
{
	ROS_INFO("RECEIVED ODOMETRY");
	this->odometry = odometry;
	return;
}

}
