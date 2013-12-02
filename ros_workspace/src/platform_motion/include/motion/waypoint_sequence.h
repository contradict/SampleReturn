#pragma once

class geometry_msgs::Point;
class std::vector;
class Eigen::Vector3d;

class WaypointSequence
{
    public:
        WaypointSequence();
        void AddWaypoint(geometry_msgs::Point waypoint);
        void Clear();

        Eigen::Vector3d GetWaypoint(unsigned int waypointIndex);
        unsigned int GetNumWaypoints();

    protected:
        std::vector<geometry_msgs::Point> waypoints;
};
