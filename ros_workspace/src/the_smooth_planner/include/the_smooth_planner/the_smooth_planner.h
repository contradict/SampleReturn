#pragma once

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

//local representation
#include <nav_core/base_local_planner.h>

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
  
private:
	ros::Publisher posePublisher;
	double maximum_velocity;
	double acceleration;
	std::vector<geometry_msgs::PoseStamped> plan;
};
};
