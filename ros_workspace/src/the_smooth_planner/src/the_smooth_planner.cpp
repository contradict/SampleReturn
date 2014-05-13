#include <the_smooth_planner/the_smooth_planner.h>
#include <the_smooth_planner/bezier_cubic_spline.h>
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
    visualization_publisher = localNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

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

    visualization_msgs::MarkerArray visualizationMarkerArray;
    visualizationMarkerArray.markers.resize(3*(path.poses.size()-1));

	// Populate the first entry with the current kinematic data.
	if (path.poses.size() > 0)
	{
		path_msg.knots[0].header.seq = 0;
		path_msg.knots[0].header.stamp = timestamp;
		path_msg.knots[0].header.frame_id = "map";
		path_msg.knots[0].pose = path.poses[0].pose;
		path_msg.knots[0].twist = odometry.twist.twist;
	}

	// Store minimum time stamps for all poses in the path ensuring that there is enough
	// time to achieve each one.  For our robot, this time is dominated by the rate at
	// which the wheels can rotate to achieve the desired wheel angle
	Eigen::Vector3d pathLinearVelocity;
	tf::vectorMsgToEigen(odometry.twist.twist.linear, pathLinearVelocity);
	double pathVelocityMagnitude = pathLinearVelocity.norm();
	BezierCubicSpline<Eigen::Vector3d>* splines = new BezierCubicSpline<Eigen::Vector3d>[path.poses.size()-1];
	for (unsigned int i = 0; i < path.poses.size()-1; ++i)
	{
		bool success = this->FitCubicSpline(path, i, pathVelocityMagnitude, maximum_linear_velocity, splines[i]);
		if (!success)
		{
			ROS_DEBUG("Failed to fit a cubic spline to the path. FIX ME!");
		}

		double distanceTraveled = splines[i].ComputeArcLength();

		// Account for acceleration rate and compute the actual velocity at the next point
		double nextVelocityMagnitude = sqrt(pathVelocityMagnitude*pathVelocityMagnitude + 2.00*linear_acceleration*distanceTraveled);
		nextVelocityMagnitude = min(nextVelocityMagnitude, maximum_linear_velocity);

		// Compute the minimum about of time necessary for the robot to traverse the
		// spline with starting and ending velocities.  This accounts for the specific
		// kinematic constraints, such as the wheel angle rotation rate limit
		double minimumPathTime = this->ComputeMinimumPathTime(splines[i], pathVelocityMagnitude, nextVelocityMagnitude);

		// Update the timestamp
		timestamp += ros::Duration(minimumPathTime);

		// Update the rviz visualization marker array
        visualizationMarkerArray.markers[3*i].header.seq = 3*i;
        visualizationMarkerArray.markers[3*i].header.frame_id = "map";
        visualizationMarkerArray.markers[3*i].header.stamp = timestamp;
        visualizationMarkerArray.markers[3*i].ns = "SmoothPlannerVisualization";
        visualizationMarkerArray.markers[3*i].id = 3*i;
        visualizationMarkerArray.markers[3*i+1].header.seq = 3*i+1;
        visualizationMarkerArray.markers[3*i+1].header.frame_id = "map";
        visualizationMarkerArray.markers[3*i+1].header.stamp = timestamp;
        visualizationMarkerArray.markers[3*i+1].ns = "SmoothPlannerVisualization";
        visualizationMarkerArray.markers[3*i+1].id = 3*i+1;
        visualizationMarkerArray.markers[3*i+2].header.seq = 3*i+2;
        visualizationMarkerArray.markers[3*i+2].header.frame_id = "map";
        visualizationMarkerArray.markers[3*i+2].header.stamp = timestamp;
        visualizationMarkerArray.markers[3*i+2].ns = "SmoothPlannerVisualization";
        visualizationMarkerArray.markers[3*i+2].id = 3*i+2;
		PopulateSplineVisualizationMarkerArray(splines[i], visualizationMarkerArray.markers[3*i], visualizationMarkerArray.markers[3*i+1], visualizationMarkerArray.markers[3*i+2]);

		// Compute the next linear velocity
		Eigen::Quaterniond nextQuaternion(path.poses[i+1].pose.orientation.w,
                                          path.poses[i+1].pose.orientation.x,
                                          path.poses[i+1].pose.orientation.y,
                                          path.poses[i+1].pose.orientation.z);
		Eigen::Vector3d nextVelocityVector = nextQuaternion * Eigen::Vector3d(1.00, 0.00, 0.00);
		nextVelocityVector *= nextVelocityMagnitude;

		// Compute the next angular velocity
		double curvature = splines[i].ComputeCurvature(1.00);
		double angularVelocity = nextVelocityMagnitude * curvature;

		// Update path velocity
		pathVelocityMagnitude = nextVelocityMagnitude;

		// Fill the outbound message data
		path_msg.knots[i+1].header.seq = i+1;
		path_msg.knots[i+1].header.stamp = timestamp;
		path_msg.knots[i+1].header.frame_id = "map";
		path_msg.knots[i+1].pose = path.poses[i+1].pose;

		path_msg.knots[i+1].twist.linear.x = nextVelocityVector.x();
		path_msg.knots[i+1].twist.linear.y = nextVelocityVector.y();
		path_msg.knots[i+1].twist.linear.z = 0.0;
		path_msg.knots[i+1].twist.angular.x = 0.0;
		path_msg.knots[i+1].twist.angular.y = 0.0;
		path_msg.knots[i+1].twist.angular.z = angularVelocity;
	}

	// Set the velocity at the last point to be zero
	path_msg.knots[path.poses.size()-1].twist.linear.x = 0.00;
	path_msg.knots[path.poses.size()-1].twist.linear.y = 0.00;
	path_msg.knots[path.poses.size()-1].twist.linear.z = 0.00;
	
	// Compute and store smooth decelerations
	for (unsigned int i = path.poses.size()-1; i > 0; --i)
	{
		Eigen::Vector3d currentVelocityVec;
		Eigen::Vector3d previousVelocityVec;
		tf::vectorMsgToEigen(path_msg.knots[i].twist.linear, currentVelocityVec);
		tf::vectorMsgToEigen(path_msg.knots[i-1].twist.linear, previousVelocityVec);
		double currentVelocityMagnitude = currentVelocityVec.norm();
		double previousVelocityMagnitude = previousVelocityVec.norm();
		double deltaTime = (path_msg.knots[i].header.stamp - path_msg.knots[i-1].header.stamp).toSec();
		if ((previousVelocityMagnitude - currentVelocityMagnitude)/deltaTime > linear_acceleration)
		{
			double previousVelocityScale = (currentVelocityMagnitude + linear_acceleration*deltaTime)/previousVelocityMagnitude;
			path_msg.knots[i-1].twist.linear.x *= previousVelocityScale;
			path_msg.knots[i-1].twist.linear.y *= previousVelocityScale;
			path_msg.knots[i-1].twist.linear.z = 0.00;
		}

	}

	if (path_msg.knots[path.poses.size()-1].twist.linear.x != 0.0 ||
        path_msg.knots[path.poses.size()-1].twist.linear.y != 0.0)
	{
		ROS_DEBUG("WHAT THE WAAAAAAAAAAAA!!!!");
	}

	// Publish path
	smooth_path_publisher.publish(path_msg);
    visualization_publisher.publish(visualizationMarkerArray);

	return;
}

void TheSmoothPlanner::setOdometry(const nav_msgs::Odometry& odometry)
{
	ROS_DEBUG("RECEIVED ODOMETRY");
	this->odometry = odometry;
	return;
}

bool TheSmoothPlanner::FitCubicSpline(const nav_msgs::Path& path,
                                      unsigned int i,
                                      double initialVelocityMagnitude,
                                      double finalVelocityMagnitude,
                                      BezierCubicSpline<Eigen::Vector3d>& outCubicSpline)
{
	const double SplineVelocityScalar = 1.0;
	
	// Create the four control points needed for the spline interpolation
	Eigen::Vector3d pointCur;
	Eigen::Vector3d pointNext;
	Eigen::Vector3d pointCurPlusVelocity;
	Eigen::Vector3d pointNextMinusVelocity;

	Eigen::Quaterniond initialQuaternion(path.poses[i].pose.orientation.w,
					 					 path.poses[i].pose.orientation.x,
									 	 path.poses[i].pose.orientation.y,
									 	 path.poses[i].pose.orientation.z);
	Eigen::Quaterniond finalQuaternion(path.poses[i+1].pose.orientation.w,
	 								   path.poses[i+1].pose.orientation.x,
									   path.poses[i+1].pose.orientation.y,
									   path.poses[i+1].pose.orientation.z);
	Eigen::Vector3d initialForwardVector = initialQuaternion * Eigen::Vector3d(1.00, 0.00, 0.00);
	Eigen::Vector3d finalForwardVector = finalQuaternion * Eigen::Vector3d(1.00, 0.00, 0.00);

	pointCur << path.poses[i].pose.position.x, path.poses[i].pose.position.y, 0.00;
	pointNext << path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y, 0.00;

	double approxDistanceBetweenPoints = (pointNext - pointCur).norm();
	//pointCurPlusVelocity = pointCur + SplineVelocityScalar*initialForwardVector*initialVelocityMagnitude;
	//pointNextMinusVelocity = pointNext - SplineVelocityScalar*finalForwardVector*finalVelocityMagnitude;
	pointCurPlusVelocity = pointCur + initialForwardVector*approxDistanceBetweenPoints/3.0;
	pointNextMinusVelocity = pointNext - finalForwardVector*approxDistanceBetweenPoints/3.0;

	// Fit a cubic spline to the control points
	outCubicSpline.SetData(pointCur,
                           pointCurPlusVelocity,
                           pointNextMinusVelocity,
                           pointNext,
                           &this->EigenVectorNorm,
                           &this->EigenVectorDot,
                           &this->EigenVectorCross);

	return true;
}

double TheSmoothPlanner::ComputeMinimumPathTime(const BezierCubicSpline<Eigen::Vector3d>& spline,
                                                double initialVelocity,
                                                double finalVelocity)
{
	double distanceTraveled = spline.ComputeArcLength(0.01);
	double averageVelocity = (initialVelocity + finalVelocity)/2.00;
	double minimumDeltaTime = distanceTraveled / averageVelocity;
	// This formula comes from a lengthy derivation in my notes. The important relation is:
	// dR/dt = -dPhi_j/dt * P_jy * csc^2(Phi_j)
	// The math here assumes the the sternPodVector is located directly
	// behind the origin (ie. has no y value)

	//double deltaCurvature = splines[i].ComputeCurvature(1.00) - splines[i].ComputeCurvature(0.99);
	//double deltaSpace = (splines[i].Interpolate(1.00) - splines[i].Interpolate(0.99)).norm();
	//double deltaCurvatureByDeltaTime = deltaCurvature/deltaSpace * nextVelocityMagnitude;
	
	//double sternAngle = atan(-sternPodVector(0)*nextCircleCurvature);
	//double sineSternAngle = sin(sternAngle);
	//double sineSternAngleSquared = sineSternAngle * sineSternAngle;
	//double requiredDeltaSternAngle;
	//if (sineSternAngleSquared > 0.0001)
	//{
	//	requiredDeltaSternAngle = deltaCurvature / (nextCircleCurvature*nextCircleCurvature) * (1.00/sternPodVector(0)) * sineSternAngle * sineSternAngle;
	//}
	//else
	//{
	//	requiredDeltaSternAngle = 0.0;
	//}
	//double minimumDeltaTime = fabs(requiredDeltaSternAngle) / maximum_slew_radians_per_second;

	return minimumDeltaTime;
}

void TheSmoothPlanner::PopulateSplineVisualizationMarkerArray(const BezierCubicSpline<Eigen::Vector3d>& spline,
                                                              visualization_msgs::Marker& marker,
                                                              visualization_msgs::Marker& pointsMarker,
                                                              visualization_msgs::Marker& circleMarker)
{
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.scale.x = 0.01; // Width of line strip
	marker.scale.y = 1.0; // NOT USED
	marker.scale.z = 1.0; // NOT USED
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	unsigned int numSteps = 100;
	marker.points.resize(numSteps+1);
	marker.colors.resize(numSteps+1);

	double tStep = 1.00/numSteps;
	unsigned int i = 0;
	for (double t = 0.00; t < 1.00+(tStep/2.00); t += tStep)
	{
		if (t > 1.00)
		{
			t = 1.00;
		}
		Eigen::Vector3d currentPoint = spline.Interpolate(t);
		marker.points[i].x = currentPoint(0);
		marker.points[i].y = currentPoint(1);
		marker.points[i].z = currentPoint(2);
		marker.colors[i].r = 1.0;
		marker.colors[i].g = 0.0;
		marker.colors[i].b = 1.0;
		marker.colors[i].a = 1.0;
		++i;
	}

	// Also draw lines for the Bezier polygon
	Eigen::Vector3d points[4];
	spline.GetData(points[0], points[1], points[2], points[3]);
	pointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
	pointsMarker.action = visualization_msgs::Marker::ADD;
	pointsMarker.pose.position.x = 0.0;
	pointsMarker.pose.position.y = 0.0;
	pointsMarker.pose.position.z = 0.0;
	pointsMarker.pose.orientation.x = 0;
	pointsMarker.pose.orientation.y = 0;
	pointsMarker.pose.orientation.z = 0;
	pointsMarker.pose.orientation.w = 1;
	pointsMarker.scale.x = 0.07;
	pointsMarker.scale.y = 0.07;
	pointsMarker.scale.z = 0.07;
	pointsMarker.color.r = 0.5;
	pointsMarker.color.g = 0.5;
	pointsMarker.color.b = 0.5;
	pointsMarker.color.a = 1.0;
	pointsMarker.points.resize(4);
	for (unsigned int i = 0; i < 4; ++i)
	{
		pointsMarker.points[i].x = points[i](0);
		pointsMarker.points[i].y = points[i](1);
		pointsMarker.points[i].z = points[i](2);
	}

    // Draw the circle of curvature
    Eigen::Vector3d tangent, normal, binormal;
    spline.ComputeTNB(1.00, tangent, normal, binormal);
    double curvature = spline.ComputeCurvature(1.00);
    if (fabs(curvature) > 0.001)
    {
        Eigen::Vector3d circleCenter = spline.Interpolate(1.00) + normal/fabs(curvature);
        circleMarker.pose.position.x = circleCenter(0);
        circleMarker.pose.position.y = circleCenter(1);
        circleMarker.pose.position.z = circleCenter(2);
        circleMarker.scale.x = 2.00/fabs(curvature);
        circleMarker.scale.y = 2.00/fabs(curvature);
    }
    else
    {
        circleMarker.pose.position.x = 10000;
        circleMarker.pose.position.y = 10000;
        circleMarker.pose.position.z = 10000;
        circleMarker.scale.x = 0.1;
        circleMarker.scale.y = 0.1;
    }
	circleMarker.type = visualization_msgs::Marker::CYLINDER;
	circleMarker.action = visualization_msgs::Marker::ADD;
	circleMarker.pose.orientation.x = 0;
	circleMarker.pose.orientation.y = 0;
	circleMarker.pose.orientation.z = 0;
	circleMarker.pose.orientation.w = 1;
	circleMarker.scale.z = 0.05;
	circleMarker.color.r = 1.0;
	circleMarker.color.g = 0.0;
	circleMarker.color.b = 1.0;
	circleMarker.color.a = 0.3;
}

}
