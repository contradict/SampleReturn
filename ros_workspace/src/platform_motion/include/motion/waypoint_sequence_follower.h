#pragma once

class Eigen::Quaterniond;
class WaypointSequence;
class ros::NodeHandle;
class geometry_msgs::PoseArray;

class WaypointSequenceFollower
{
    public:
        WaypointSequenceFollower();

    protected:
        // Message functions
        void HandleNewWaypointSequence(geometry_msgs::PoseArray poseArray);

        // Helper functions
        Eigen::Quaterniond GetOrientationBetweenWaypoints(unsigned int waypointIndex1, unsigned int waypointIndex2);
        double GetDistanceBetweenWaypoints(unsigned int waypointIndex1, unsigned int waypointIndex2);

        ros::NodeHandle parentNodeHandle;
        ros::NodeHandle pubSubNodeHandle;

        WaypointSequence waypointSequence;

        double maxLinearVelocity;
        double maxAngularVelocity;
        double maxLinearAcceleration;
        double maxAngularAcceleration;
};
