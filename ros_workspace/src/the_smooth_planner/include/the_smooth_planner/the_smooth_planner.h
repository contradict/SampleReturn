#pragma once

#include <iostream>
#include <vector>

/** ROS **/
#include <Eigen/Dense>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <platform_motion_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <the_smooth_planner/bezier_cubic_spline.h>

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

    virtual bool requestNewPlanFrom(geometry_msgs::PoseStamped* sourcePose);

    virtual bool isGoalReached();

    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    void setPath(const nav_msgs::Path& path);
    void setOdometry(const nav_msgs::Odometry& odometry);
    void setCompletedKnot(const std_msgs::Header& completedKnot);
    void setMaximumVelocity(const std_msgs::Float64::ConstPtr velocity);
    void setStitchedPath(const platform_motion_msgs::Path& stitchedPath);
  
private:
    static double EigenVectorNorm(const Eigen::Vector3d& vec) { return vec.norm(); }
    static double EigenVectorDot(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) { return v1.dot(v2); }
    static Eigen::Vector3d EigenVectorCross(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) { return v1.cross(v2); }

    bool FitCubicSpline(const nav_msgs::Path& path,
                        unsigned int i,
                        double initialVelocityMagnitude,
                        double finalVelocityMagnitude,
                        BezierCubicSpline<Eigen::Vector3d>& outCubicSpline);

    double ComputeCurvature(const nav_msgs::Path& path, unsigned int i);

    double ComputeMinimumPathTime(const nav_msgs::Path& path, unsigned int i);   

    double ComputeMinimumPathTime(const BezierCubicSpline<Eigen::Vector3d>& spline,
                                  double initialVelocity,
                                  double finalVelocity);

    void PopulateSplineVisualizationMarkerArray(const BezierCubicSpline<Eigen::Vector3d>& spline,
                                                visualization_msgs::Marker& marker,
                                                visualization_msgs::Marker& pointsMarker,
                                                visualization_msgs::Marker& circleMarker);


    // Publishers and subscribers
    ros::Publisher smooth_path_publisher;
    ros::Publisher visualization_publisher;
    ros::Subscriber pose_subscriber;
    ros::Subscriber odom_subscriber;
    ros::Subscriber completed_knot_subscriber;
    ros::Subscriber stitched_path_subscriber;
    ros::Subscriber max_velocity_subscriber;

    // Parameters
    double maximum_linear_velocity;
    double linear_acceleration;
    double maximum_slew_radians_per_second;
    double replan_look_ahead_buffer_time;
    double replan_look_ahead_time;
    double yaw_epsilon;
    double delta_time_after_goal_drop_path;

    // Message data
    std_msgs::Header completed_knot_header;
    nav_msgs::Odometry odometry;
    Eigen::Vector3d sternPodVector;
    platform_motion_msgs::Path stitched_path;
    std::vector<platform_motion_msgs::Knot>::iterator replan_ahead_iter;
    bool is_replan_ahead_iter_valid;
};
};
