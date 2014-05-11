#pragma once

#include <iostream>
#include <vector>

/** ROS **/
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <the_smooth_planner/circle.h>

namespace the_smooth_planner
{

class TheSmoothPlanner : public nav_core::BaseLocalPlanner
{
public:
	TheSmoothPlanner();
	TheSmoothPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
	virtual ~TheSmoothPlanner() {};

	virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

	virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

	virtual bool isGoalReached();

	virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

	void setPath(const nav_msgs::Path& path);
	void setOdometry(const nav_msgs::Odometry& odometry);
  
private:
	double ComputeMinimumPathTime(const nav_msgs::Path& path,
								  unsigned int i,
                                  double& distanceTraveled,
                                  Circle& outCircle);

	ros::Publisher smooth_path_publisher;
	ros::Publisher visualization_publisher;
	ros::Subscriber pose_subscriber;
	ros::Subscriber odom_subscriber;
	double maximum_linear_velocity;
	double linear_acceleration;
	double maximum_slew_radians_per_second;
	nav_msgs::Odometry odometry;
	Eigen::Vector3d sternPodVector;
};
};
