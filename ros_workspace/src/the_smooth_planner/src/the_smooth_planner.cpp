#include <the_smooth_planner/the_smooth_planner.h>
#include <the_smooth_planner/circle.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/inflation_layer.h>
#include <platform_motion_msgs/Path.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
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
	localNodeHandle.param("maximum_slew_radians_per_second", maximum_slew_radians_per_second, 0.3);

	this->odometry = nav_msgs::Odometry();

	pose_subscriber = parentNodeHandle.subscribe("plan", 1, &TheSmoothPlanner::setPath, this);
	odom_subscriber = parentNodeHandle.subscribe("odometry", 1, &TheSmoothPlanner::setOdometry, this);

	smooth_path_publisher = localNodeHandle.advertise<platform_motion_msgs::Path>("/motion/planned_path", 1);

	// Wait for and look up the transform for the stern wheel pod
	ROS_INFO("Waiting for stern wheel transform");
	ros::Time now(0);
	ros::Duration waitDuration(1.0);
	while (!tf->waitForTransform("base_link", "stern_suspension", now, waitDuration) && ros::ok())
	{
		ROS_INFO("Waiting for stern wheel transform");
	}
	if (ros::ok())
	{
		ROS_INFO("Received stern wheel transform");
	}

	now = ros::Time(0);
	tf::StampedTransform sternPodStampedTf;
	try
	{
		tf->lookupTransform("base_link", "stern_suspension", now, sternPodStampedTf);
	}
	catch (tf::TransformException e)
	{
		ROS_ERROR("Error looking up %s: %s", "stern_suspension", e.what());
	}
	tf::vectorTFToEigen(sternPodStampedTf.getOrigin(), sternPodVector);
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

	platform_motion_msgs::Path path_msg;
	path_msg.knots.resize(path.poses.size());
	path_msg.header.stamp = timestamp;
	path_msg.header.seq = 0;
	path_msg.header.frame_id = "map";

	// Populate the first entry with the current kinematic data.
	if (path.poses.size() > 0)
	{
		path_msg.knots[0].header.seq = 0;
		path_msg.knots[0].header.stamp = timestamp;
		path_msg.knots[0].header.frame_id = "map";
		path_msg.knots[0].pose = path.poses[0].pose;
		path_msg.knots[0].twist = odometry.twist.twist;
	}

	// Store time stamps for all poses in the path ensuring that there is enough
	// time to achieve each one
	for (unsigned int i = 0; i < path.poses.size()-1; ++i)
	{
		// Compute the minimum allowable time between this path point and the next based on the path curvature and the slew
		// rate limit of the wheel pod servos
		tf::Vector3 initialLinearVelocityDirection(path_msg.knots[i].twist.linear.x, path_msg.knots[i].twist.linear.y, 0.0);
		double minimumPathTime = this->ComputeMinimumPathTime(path, i);

		timestamp += ros::Duration(minimumPathTime);

		// Fill the outbound message data
		path_msg.knots[i+1].header.seq = i+1;
		path_msg.knots[i+1].header.stamp = timestamp;
		path_msg.knots[i+1].header.frame_id = "map";
		path_msg.knots[i+1].pose = path.poses[i+1].pose;
	}

	// Compute and store velocities with smooth acceleration up to the maximum linear velocity
	Eigen::Vector3d linearVelocity(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z);
	Eigen::Vector3d angularVelocity(odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z);
	double currentVelocityMagnitude = linearVelocity.norm(); // Current robot velocity
	double pathVelocityMagnitude = currentVelocityMagnitude; // Used to accumulate velocity changes over the planned path
	double distanceTraveled = 0.0;
	Time* actualTimestamps = new Time[path.poses.size()];
	double* actualVelocities = new double[path.poses.size()];
	if (path.poses.size() > 0)
	{
		actualTimestamps[0] = path_msg.knots[0].header.stamp;
		actualVelocities[0] = sqrt(path_msg.knots[0].twist.linear.x*path_msg.knots[0].twist.linear.x +
								   path_msg.knots[0].twist.linear.y*path_msg.knots[0].twist.linear.y);
	}
	for (unsigned int i = 0; i < path.poses.size()-1; ++i)
	{
		// Approximate the distance to the next point in the path linearly.  This underestimates the actual distance
		// that will be traveled, but will lead to slower velocities, which is acceptable
		double deltaX = (path.poses[i+1].pose.position.x - path.poses[i].pose.position.x);
		double deltaY = (path.poses[i+1].pose.position.y - path.poses[i].pose.position.y);
		double linear_distance = sqrt(deltaX*deltaX + deltaY*deltaY);
		distanceTraveled += linear_distance;

		// Compute the maximum attainable average velocity required to get to the next point with the current time stamps, which
		// completely ignore linear acceleration constraints.  The velocity is a maximum because the time stamps assigned in the
		// previous step were minimum times
		double deltaTime = (path.poses[i+1].header.stamp - path.poses[i].header.stamp).toSec();
		double maximumAttainableAverageVelocity = std::numeric_limits<double>::infinity();
		if (deltaTime > 0)
		{
			maximumAttainableAverageVelocity = linear_distance / deltaTime;
		}

		// Use the biggest velocity not bigger than the maximum linear velocity
		double desiredAverageVelocity = min(maximumAttainableAverageVelocity, maximum_linear_velocity);

		// Account for acceleration rate and compute the actual velocity at the next point and
		// save it for later use
		pathVelocityMagnitude = sqrt(pathVelocityMagnitude*pathVelocityMagnitude + 2.00*linear_acceleration*distanceTraveled);
		pathVelocityMagnitude = min(pathVelocityMagnitude, desiredAverageVelocity);
		actualVelocities[i+1] = pathVelocityMagnitude;

		// Recompute the amount of time required to get to the next point and save the
		// timestamp in a separate array so we don't overwrite the time stamps in the message
		deltaTime = linear_distance / pathVelocityMagnitude;
		actualTimestamps[i+1] = actualTimestamps[i] + ros::Duration(deltaTime);

		// Compute and store the angular rates in the outbound message
		tf::Quaternion initialQuaternion(path.poses[i].pose.orientation.x,
										 path.poses[i].pose.orientation.y,
										 path.poses[i].pose.orientation.z,
										 path.poses[i].pose.orientation.w);
		tf::Quaternion finalQuaternion(path.poses[i+1].pose.orientation.x,
									   path.poses[i+1].pose.orientation.y,
									   path.poses[i+1].pose.orientation.z,
									   path.poses[i+1].pose.orientation.w);
		tf::Quaternion initialToFinalQuaternion = initialQuaternion.inverse()*finalQuaternion;
		double angle = tf::getYaw(initialToFinalQuaternion);
		double angularVelocity = angle / deltaTime;
		path_msg.knots[i+1].twist.angular.x = 0.0;
		path_msg.knots[i+1].twist.angular.y = 0.0;
		path_msg.knots[i+1].twist.angular.z = angularVelocity;

		// Compute and store unit-length linear velocities in the outbound message.  This
		// will be scaled in the next step
		tf::Vector3 forwardVector(1, 0, 0);
		tf::Vector3 finalLinearDirection = tf::quatRotate(finalQuaternion, forwardVector);
		path_msg.knots[i+1].twist.linear.x = finalLinearDirection.x();
		path_msg.knots[i+1].twist.linear.y = finalLinearDirection.y();
		path_msg.knots[i+1].twist.linear.z = 0.0;
	}

	// Set the actual timestamps for the outbound message
	for (unsigned int i = 0; i < path.poses.size(); ++i)
	{
		path_msg.knots[i].header.stamp = actualTimestamps[i];
	}

	// Set the velocity at the last point to be zero
	actualVelocities[path.poses.size()-1] = 0.00;
	
	// Compute and store smooth decelerations to replace the velocity dicontinuities created
	// in the previous step
	for (unsigned int i = path.poses.size()-1; i > 0; --i)
	{
		double currentVelocity = actualVelocities[i];
		double previousVelocity = actualVelocities[i-1];
		double deltaTime = (path_msg.knots[i].header.stamp - path_msg.knots[i-1].header.stamp).toSec();
		if ((previousVelocity - currentVelocity)/deltaTime > linear_acceleration)
		{
			actualVelocities[i-1] = actualVelocities[i] + linear_acceleration*deltaTime;
		}

		path_msg.knots[i-1].twist.linear.x *= actualVelocities[i-1];
		path_msg.knots[i-1].twist.linear.y *= actualVelocities[i-1];
	}
	path_msg.knots[0].twist = odometry.twist.twist;

	// Free up memory
	delete [] actualTimestamps;
	delete [] actualVelocities;

	// Publish path
	smooth_path_publisher.publish(path_msg);

	return;
}

void TheSmoothPlanner::setOdometry(const nav_msgs::Odometry& odometry)
{
	ROS_DEBUG("RECEIVED ODOMETRY");
	this->odometry = odometry;
	return;
}

double TheSmoothPlanner::ComputeMinimumPathTime(const nav_msgs::Path& path,
												unsigned int i)
{
	// This formula comes from a lengthy derivation in my notes. The important relation is:
	// dR/dt = -dPhi_j/dt * P_jy * csc^2(Phi_j)
	// The math here assumes the the sternPodVector is located directly
	// behind the origin (ie. has no y value)

	Eigen::Vector2d pointPrev;
	Eigen::Vector2d pointCur;
	Eigen::Vector2d pointNext;
	Eigen::Vector2d pointAfterNext;
	if (i > 0 && i < path.poses.size() - 2)
	{
		pointPrev << path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y;
		pointCur << path.poses[i].pose.position.x, path.poses[i].pose.position.y;
		pointNext << path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y;
		pointAfterNext << path.poses[i+2].pose.position.x, path.poses[i+2].pose.position.y;
	}
	else if (i == 0)
	{
		// TODO - Handle this later.  First try planning from stop to stop assuming
		//        the pointPrev is co-linear with pointCur and pointNext.  Later,
		//        have this planner track the whole plan so it can splice in
		//        the new path request and look up existing point data
		Eigen::Quaterniond quatPrev;
		tf::quaternionMsgToEigen(path.poses[i].pose.orientation, quatPrev);
		Eigen::Vector3d forwardDirPrev = quatPrev * Eigen::Vector3d(1.00, 0.00, 0.00);
		pointPrev << path.poses[i].pose.position.x, path.poses[i].pose.position.y;
		pointPrev(0) -= forwardDirPrev(0);
		pointPrev(1) -= forwardDirPrev(1);

		pointCur << path.poses[i].pose.position.x, path.poses[i].pose.position.y;
		pointNext << path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y;
		pointAfterNext << path.poses[i+2].pose.position.x, path.poses[i+2].pose.position.y;
	}
	else if (i == path.poses.size() - 2)
	{
		// Make the end of the path segment straight, with and infinite radius
		pointPrev << path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y;
		pointCur << path.poses[i].pose.position.x, path.poses[i].pose.position.y;
		pointNext << path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y;

		Eigen::Quaterniond quatNext;
		tf::quaternionMsgToEigen(path.poses[i+1].pose.orientation, quatNext);
		Eigen::Vector3d forwardDirNext = quatNext * Eigen::Vector3d(1.00, 0.00, 0.00);
		pointAfterNext << path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y;
		pointAfterNext(0) += forwardDirNext(0);
		pointAfterNext(1) += forwardDirNext(1);
	}
	else if (i == path.poses.size() - 1)
	{
		// Make the end of the path segment straight, with and infinite radius
		pointPrev << path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y;
		pointCur << path.poses[i].pose.position.x, path.poses[i].pose.position.y;

		Eigen::Quaterniond quatCur;
		tf::quaternionMsgToEigen(path.poses[i].pose.orientation, quatCur);
		Eigen::Vector3d forwardDirCur = quatCur * Eigen::Vector3d(1.00, 0.00, 0.00);
		pointNext << path.poses[i].pose.position.x, path.poses[i].pose.position.y;
		pointNext(0) += forwardDirCur(0);
		pointNext(1) += forwardDirCur(1);
		pointAfterNext << path.poses[i].pose.position.x, path.poses[i].pose.position.y;
		pointAfterNext(0) += 2.0*forwardDirCur(0);
		pointAfterNext(1) += 2.0*forwardDirCur(1);
	}

	Circle prevCircle(pointPrev, pointCur, pointNext);
	Circle nextCircle(pointCur, pointNext, pointAfterNext);

	Eigen::Vector3d pointPrev3d(pointPrev(0), pointPrev(1), 0);
	Eigen::Vector3d pointCur3d(pointCur(0), pointCur(1), 0);
	Eigen::Vector3d pointNext3d(pointNext(0), pointNext(1), 0);
	Eigen::Vector3d prevCircleCenter(prevCircle.GetCenterX(), prevCircle.GetCenterY(), 0);
	Eigen::Vector3d nextCircleCenter(nextCircle.GetCenterX(), nextCircle.GetCenterY(), 0);
	Eigen::Vector3d vectorToPrevCenter = prevCircleCenter - pointPrev3d;
	Eigen::Vector3d centerToNextCenter = nextCircleCenter - pointCur3d;
	double prevCircleCurvature = prevCircle.GetCurvature();
	double nextCircleCurvature = nextCircle.GetCurvature();
	if ((pointCur3d - pointPrev3d).cross((prevCircleCenter - pointPrev3d))(2) < 0)
	{
		prevCircleCurvature = -prevCircleCurvature;
	}
	if ((pointNext3d - pointCur3d).cross((nextCircleCenter - pointCur3d))(2) < 0)
	{
		nextCircleCurvature = -nextCircleCurvature;
	}

	double deltaCurvature = nextCircleCurvature - prevCircleCurvature;
	double sternAngle = atan(-sternPodVector(0)*nextCircleCurvature);
	double sineSternAngle = sin(sternAngle);
	double sineSternAngleSquared = sineSternAngle * sineSternAngle;
	double requiredDeltaSternAngle;
	if (sineSternAngleSquared > 0.0001)
	{
		requiredDeltaSternAngle = deltaCurvature / (nextCircleCurvature*nextCircleCurvature) * (1.00/sternPodVector(0)) * sineSternAngle * sineSternAngle;
	}
	else
	{
		requiredDeltaSternAngle = 0.0;
	}
	double minimumDeltaTime = requiredDeltaSternAngle / maximum_slew_radians_per_second;
		
	return minimumDeltaTime;
}

}
