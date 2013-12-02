#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include "waypoint_sequence_follower.h"

WaypointSequenceFollower::WaypointSequenceFollower() : parentNodeHandle("~")
{
    parentNodeHandle.param<double>("max_linear_velocity", maxLinearVelocity, 2.0);
    parentNodeHandle.param<double>("max_angular_velocity", maxAngularVelocity, 1.5);
    parentNodeHandle.param<double>("max_linear_acceleration", maxLinearAcceleration, 5.0);
    parentNodeHandle.param<double>("max_angular_acceleration", maxAngularAcceleration, 3.0);

    pubSubNodeHandle.subscribe("waypoint_sequence", &WaypointSequenceFollower::HandleNewWaypointSequence, this);
    pubSubNodeHandle.advertise<geometry_msgs::Twist>("waypoint_sequence_follow_commands", 1); // TODO - make this advertise type an array of twists, possibly including timestamps
}

void WaypointSequenceFollower::HandleNewWaypointSequence(geometry_msgs::PoseArray poseArray)
{
    // TODO - Implement me!
    // 1) Get to the first waypoint (are we already there?)
    // 2) Compute the orientation of the first line in the piecewise linear waypoint path, using helper func
    // 3) Compute twist interpolation commands to send to acheive the target orientation
    // 4) Compute the distance to the next waypoint, using helper func
    // 5) Compute twist interpolation commands to send to achieve the target pose
    // 6) Aggregate all commands and publish them all simultaneously (is this a good idea?)
}

Eigen::Quaterniond WaypointSequenceFollower::GetOrientationBetweenWaypoints(unsigned int waypointIndex1, unsigned int waypointIndex2)
{
    Eigen::Vector3d waypoint1 = waypointSequence.GetWaypoint(waypointIndex1);
    Eigen::Vector3d waypoint2 = waypointSequence.GetWaypoint(waypointIndex2);

    Eigen::Vector3d toWaypoint2 = waypoint2 - waypoint1;
    Eigen::Vector3d mapFrameForward(1, 0, 0); // TODO - Is this the right vector for computing angles?

    Eigen::Quaternion3d orientationToWaypoint2;
    orientationToWaypoint2.FromTwoVectors(mapFrameForward, toWaypoint2);

    return orientationToWaypoint2;
}

double GetDistanceBetweenWaypoints(unsigned int waypointIndex1, unsigned int waypointIndex2)
{
    Eigen::Vector3d waypoint1 = waypointSequence.GetWaypoint(waypointIndex1);
    Eigen::Vector3d waypoint2 = waypointSequence.GetWaypoint(waypointIndex2);

    Eigen::Vector3d toWaypoint2 = waypoint2 - waypoint1;
    return toWaypoint2.norm();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "WaypointSequenceFollower");

    WaypointSequenceFollower waypointSequenceFollower;

    ros::spin();
}
