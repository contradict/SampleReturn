#include <geometry_msgs/Point.h>
#include <vector>
#include <Eigen/Dense>
#include "waypoint_sequence.h"

WaypointSequence::WaypointSequence()
{
}

void WaypointSequence::AddWaypoint(geometry_msgs::Point waypoint)
{
    waypoints.push_back(Eigen::Vector3d(waypoint.x, waypoint.y, waypoint.z));
}

void WaypointSequence::Clear()
{
    waypoints.clear();
}

Eigen::Vector3d GetWaypoint(unsigned int waypointIndex)
{
    return waypoints[waypointIndex];
}

unsigned int GetNumWaypoints()
{
    return waypoints.size();
}
