#include <the_smooth_planner/the_smooth_planner.h>
#include <the_smooth_planner/bezier_cubic_spline.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/inflation_layer.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <tuple>  // this is going to make you sad, trust me.
#include <functional>

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
    ros::NodeHandle parentNodeHandle("~");

    localNodeHandle.param("maximum_linear_velocity", maximum_linear_velocity, 1.0);
    localNodeHandle.param("linear_acceleration", linear_acceleration, 1.0);
    localNodeHandle.param("replan_look_ahead_buffer_time", replan_look_ahead_buffer_time, 1.0);
    localNodeHandle.param("replan_look_ahead_time", replan_look_ahead_time, 4.0);
    localNodeHandle.param("delta_time_after_goal_drop_path", delta_time_after_goal_drop_path, 20.0);
    localNodeHandle.param("wait_on_stitched_path_duration", wait_on_stitched_path_duration, 20.0);
    localNodeHandle.param("xy_goal_tolerance", xy_goal_tolerance, 0.4);


    this->odometry = nav_msgs::Odometry();
    this->completed_knot_header = std_msgs::Header();
    this->replan_ahead_iter = this->stitched_path.knots.begin();
    this->is_replan_ahead_iter_valid = false;
    this->is_waiting_on_stitched_path = false;
    this->is_goal_reached = false;
    this->have_goal = false;
    this->start_time_wait_on_stitched_path = Time::now();

    /*
    this->replan_ahead_pose.position.x = 0;
    this->replan_ahead_pose.position.y = 0;
    this->replan_ahead_pose.position.z = 0;
    this->replan_ahead_pose.orientation.x = 0;
    this->replan_ahead_pose.orientation.y = 0;
    this->replan_ahead_pose.orientation.z = 0;
    this->replan_ahead_pose.orientation.w = 0;
    */

    odom_subscriber = parentNodeHandle.subscribe("odometry", 1, &TheSmoothPlanner::setOdometry, this);
    completed_knot_subscriber = parentNodeHandle.subscribe("completed_knot", 1, &TheSmoothPlanner::setCompletedKnot, this);
    stitched_path_subscriber = parentNodeHandle.subscribe("stitched_path", 1, &TheSmoothPlanner::setStitchedPath, this);
    max_velocity_subscriber = parentNodeHandle.subscribe("max_velocity", 1, &TheSmoothPlanner::setMaximumVelocity, this);

    smooth_path_publisher = localNodeHandle.advertise<platform_motion_msgs::Path>("/motion/planned_path", 1);
    visualization_publisher = localNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    desired_stitch_point_publisher = localNodeHandle.advertise<geometry_msgs::PoseStamped>("/motion/desired_stitch_point", 1);
    buffer_point_publisher = localNodeHandle.advertise<geometry_msgs::PoseStamped>("/motion/buffer_point", 1);

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

    ROS_DEBUG("sternPodVector.norm: %f", sternPodVector[0]);

    timeSinceLastPlanFromPlanner = Time::now();
    // NOTE: this should probably be a parameter, and smaller!
    plannerTimeoutDuration = ros::Duration(2.0);
}

bool TheSmoothPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    // This is handled in the pvt_segment code instead of this local planner
    // Just putzeros here so the value is sensible
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    // return false if we've not heard from the planner in a while, which is real bad!
    if((Time::now() - timeSinceLastPlanFromPlanner) > plannerTimeoutDuration)
    {
        // return false, this should cause the goal to fail
        return false;
    }
    return true;
}

bool TheSmoothPlanner::requestNewPlanFrom(geometry_msgs::PoseStamped* sourcePose)
{
    // we heard from the planner. that's good.
    timeSinceLastPlanFromPlanner = Time::now();
    if (this->is_waiting_on_stitched_path)
    {
        ROS_ERROR("waiting on stitched path, ignoring replan request");
        return false;
    }
    else if (this->isGoalReached() || is_replan_ahead_iter_valid)
    {
        ROS_ERROR("last path has no knots or we reached the goal or replan iter is valid");
        return false;
    }
    else if ( stitched_path.knots.size() == 0 && have_goal )
    {
        geometry_msgs::PoseStamped odometry_pose;
        odometry_pose.header = odometry.header;
        odometry_pose.pose = odometry.pose.pose;
        try {
            listener.transformPose("map", odometry_pose, *sourcePose );
            return true;
        } catch( tf::TransformException ex) {
            ROS_ERROR("Error transforming request into map: %s", ex.what());
        }
    }
    else if( stitched_path.knots.size() != 0 )
    {
        ROS_ERROR("last path has knots and we haven't reached the goal");
        auto currentKnotIter = stitched_path.knots.begin();
        for (; currentKnotIter != stitched_path.knots.end(); ++currentKnotIter)
        {
            if ((*currentKnotIter).header.seq == completed_knot_header.seq &&
                fabs(((*currentKnotIter).header.stamp - completed_knot_header.stamp).toSec()) < 0.01)
            {
                break;
            }
        }
        if (currentKnotIter != stitched_path.knots.end())
        {
            ROS_ERROR("current knot is not the end of last path");
            // See how much time until the expected end of the current plan
            ros::Duration timeUntilPlanEnd = stitched_path.knots.back().header.stamp - (*currentKnotIter).header.stamp;
            double distanceUntilPlanEnd = hypot((stitched_path.knots.back().pose.position.x - (*currentKnotIter).pose.position.x),
                    (stitched_path.knots.back().pose.position.y - (*currentKnotIter).pose.position.y));
            if (timeUntilPlanEnd < ros::Duration(replan_look_ahead_time))
            {
                ROS_ERROR("time until plan end is too short");
                return false; // Finish the current plan
            }
            else if( distanceUntilPlanEnd < xy_goal_tolerance )
            {
                ROS_ERROR("distance until plan end is too short");
                return false;
            }
            else
            {
                ROS_ERROR("time until plan end fine");
                // get the position of the current knot and check the potential stitch point knots to make sure they're far enough away.
                Eigen::Vector3d currentPos;
                tf::pointMsgToEigen((*currentKnotIter).pose.position, currentPos);

                // Request a plan from a point ahead in the current path to
                // the same goal
                for (auto searchAheadIter = currentKnotIter; searchAheadIter != stitched_path.knots.end(); ++searchAheadIter)
                {
                    ros::Duration aheadTime = (*searchAheadIter).header.stamp - (*currentKnotIter).header.stamp;
                    ROS_DEBUG_STREAM("Current seq: " << (*currentKnotIter).header.seq << ", Search seq: " << (*searchAheadIter).header.seq << ", Ahead time: " << aheadTime);

                    Eigen::Vector3d searchAheadPos, searchAheadPrevPos;
                    Eigen::Quaterniond searchAheadQuat, searchAheadPrevQuat;

                    tf::pointMsgToEigen((*searchAheadIter).pose.position, searchAheadPos);
                    tf::quaternionMsgToEigen((*searchAheadIter).pose.orientation, searchAheadQuat);
                    if (currentKnotIter != stitched_path.knots.begin())
                    {
                        tf::pointMsgToEigen((*(searchAheadIter-1)).pose.position, searchAheadPrevPos);
                        tf::quaternionMsgToEigen((*(searchAheadIter-1)).pose.orientation, searchAheadPrevQuat);
                    }
                    else
                    {
                        continue;
                    }
                    
                    bool isAheadEnoughInTime = aheadTime > ros::Duration(replan_look_ahead_time);
                    bool isAheadEnoughInDistance = (searchAheadPos - currentPos).norm() >= (replan_look_ahead_time * maximum_linear_velocity);
                    bool isPosApproxPrev = (searchAheadPos - searchAheadPrevPos).squaredNorm() < 0.0022;
                    bool isQuatApproxPrev = searchAheadQuat.dot(searchAheadPrevQuat) > 0.9952; // 0.9952 = cos(11.25/2.0 deg)
                    ROS_DEBUG_STREAM("DesiredPose - isAheadEnoughInTime: " << isAheadEnoughInTime << "isAheadEnoughInDistance: " << isAheadEnoughInDistance << ", IsPosApproxPrev: " << isPosApproxPrev << ", IsQuatApproxPrev: " << isQuatApproxPrev);
                    if (isAheadEnoughInTime &&  isAheadEnoughInDistance && !isPosApproxPrev && isQuatApproxPrev)
                    {
                        ROS_ERROR_STREAM("Choosing stitch point: \n" << (*searchAheadIter));
                        sourcePose->pose = (*searchAheadIter).pose;
                        sourcePose->header = (*searchAheadIter).header;
                        this->replan_ahead_iter = searchAheadIter;
                        this->is_replan_ahead_iter_valid = true;
                        desired_stitch_point_publisher.publish(*sourcePose);
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

bool TheSmoothPlanner::isGoalReached()
{
    // The pvt_segment code in platform_motion knows how to answer this
    // question much better than the local planner.  The goal is reached
    // when all pvt segments have been executed to completion.
    return is_goal_reached;
}

bool TheSmoothPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    bool retVal = true;
    // The global planner emits a nav_msgs::Path instead of this message type.
    // We reuse the nav_msgs::Path handler here
    if (plan.size() > 0)
    {
        nav_msgs::Path path;
        path.poses = plan;
        path.header = plan[0].header;
        // set this time here so we can use the setPath method later
        timeSinceLastPlanFromPlanner = Time::now();
        retVal = this->setPath(path);
    }
    ROS_DEBUG("RECEIVED PLAN");
    return retVal;
}

double yawFromKnot(const platform_motion_msgs::Knot &knot)
{
    return 2.0*atan2(knot.pose.orientation.z, knot.pose.orientation.w);
}

double yawFromMsgQuat(const geometry_msgs::Quaternion& quat)
{
    return 2.0*atan2(quat.z, quat.w);
}

void yawToKnot(platform_motion_msgs::Knot &knot, double yaw)
{
    knot.pose.orientation.w = cos(yaw/2.0);
    knot.pose.orientation.z = sin(yaw/2.0);
}

std::tuple<double, std::function<double(double)>, std::function<double(double)> > scurve(double initial, double final, double dotmax)
{
    double T = fabs(final-initial)*3.0/2.0/dotmax;
    auto x = [=](double s) -> double {return initial+(final-initial)*3/2.0*s*s*(.5-s/3)*4.0;};
    auto xdot = [=](double s) -> double {return (final-initial)*3.0/2.0*s*(1.0-s)*4.0/T;};
    return std::make_tuple(T, x, xdot);
}

void moveR(std::vector<platform_motion_msgs::Knot> &path, platform_motion_msgs::Knot knot, int N, double R0, double R1, double phidot, double omegasmall, double vsmall, double wheelbase)
{
    double phi0 = M_PI/2.0-atan(R0/wheelbase);
    double phi1 = M_PI/2.0-atan(R1/wheelbase);
    double yaw = yawFromKnot(knot);

    double T = fabs(phi1-phi0)*3.0/2.0/phidot;
    auto phi = [=](double s) -> double {return phi0+(phi1-phi0)*3/2.0*s*s*(.5-s/3)*4.0;};

    Eigen::Vector2d P(knot.pose.position.x, knot.pose.position.y);
    Eigen::Vector2d V(knot.twist.linear.x, knot.twist.linear.y);
    for(int x = 1; x < N; x++)
    {
        double s = ((double)x)/(N-1);
        double t = s*T;
        double R = wheelbase*tan(M_PI/2.0-phi(s));
        double v, omega;
        if(R>0)
        {
            omega = min(omegasmall, vsmall/R);
            v = omega*R;
        }
        else
        {
            omega = omegasmall;
            v = 0.0;
        }
        yaw += omega*T/N;
        Eigen::Vector2d Vprime(cos(yaw), sin(yaw));
        Vprime = Vprime * v;
        P += (V*T/N + Vprime*T/N)/2.0;
        V = Vprime;
        yawToKnot(knot, yaw);
        knot.pose.position.x = P[0];
        knot.pose.position.y = P[1];
        knot.twist.angular.z = omega;
        knot.twist.linear.x = V[0];
        knot.twist.linear.y = V[1];
        knot.header.stamp += ros::Duration(T/N);
        knot.header.seq += 1;
        path.push_back(knot);
    }
}

void rotate(std::vector<platform_motion_msgs::Knot> &path, platform_motion_msgs::Knot knot, int N, double yaw1, double omegapeak)
{
    double yaw0 = yawFromKnot(knot);
    // keep from going the long way around...
    if((yaw0 -yaw1) < -M_PI)
    {
        yaw1 -= 2.0*M_PI;
    }
    else if((yaw0 -yaw1) > M_PI)
    {
        yaw1 += 2.0*M_PI;
    }
    double T = fabs(yaw1-yaw0)*3.0/2.0/omegapeak;
    auto yaw = [=](double s) -> double {return yaw0+(yaw1-yaw0)*3/2.0*s*s*(.5-s/3)*4.0;};
    auto omega = [=](double s) -> double {return (yaw1-yaw0)*3.0/2.0*s*(1.0-s)*4.0/T;};

    for(int x = 1; x < N; x++)
    {
        double s = ((double)x)/(N-1.0);
        double t = T*s;
        yawToKnot(knot, yaw(s));
        knot.twist.linear.x = 0;
        knot.twist.linear.y = 0;
        knot.twist.angular.z = omega(s);
        knot.header.stamp += ros::Duration(T/N);
        knot.header.seq += 1;
        path.push_back(knot);
    }
}

std::vector<platform_motion_msgs::Knot> computeTurnInPlace(platform_motion_msgs::Knot &start, platform_motion_msgs::Knot &end, ros::Duration &turnDuration, Eigen::Vector3d sternPodVector)
{
    std::vector<platform_motion_msgs::Knot> retval;

    // some magic numbers from the original python:
    int numPoints = 20;
    double startR = 1e5;
    double endR = 0.0;
    double wheelbase = fabs(sternPodVector[0]);
    double phidot = M_PI/2.0;
    double omegasmall = 3e-3;
    double vsmall = 3e-3;
    double omegapeak = M_PI/8.0;

    // compute the delta yaw we're going through
    double startingYaw = yawFromKnot(start);
    double endingYaw = yawFromKnot(end);

    if(fabs(startingYaw - endingYaw) <= M_PI/360.0)
    {
        // if we're not actually turning, that's bad. just return
        turnDuration = ros::Duration(0.0);
        return retval;
    }

    // use moveR to get the stern wheel pointing in the right direction
    moveR(retval, start, numPoints, startR, endR, phidot, omegasmall, vsmall, wheelbase);

    // actually rotate
    rotate(retval, retval.back(), numPoints, endingYaw, omegapeak);

    // use moveR to get the stern wheel pointing straight again.
    moveR(retval, retval.back(), numPoints, endR, startR, phidot, omegasmall, vsmall, wheelbase);

    // duration is just the difference between the last time in retval and the time in start
    turnDuration = retval.back().header.stamp - start.header.stamp;

    return retval;
}

// Returns false when it rejects the plans and wants it to be republished later
bool TheSmoothPlanner::setPath(const nav_msgs::Path& path)
{
    ROS_ERROR("RECEIVED PATH");

    Time timestamp = Time::now();

    if ( (is_waiting_on_stitched_path && (timestamp-start_time_wait_on_stitched_path) < ros::Duration(wait_on_stitched_path_duration)) ||
          path.poses.size() == 0 ) 
    {
        ROS_ERROR("WAITING ON STITCHED PATH, IGNORING NEW PLAN");
        return true;
    }

    // Poses to remove for turn-in-place
    std::vector<int> posesToRemove;

    // make a path copy to deal with stupid const stupid
    nav_msgs::Path pathCopy(path);

    // debug print! print all the points, something is strange about them!
    for(unsigned int i = 0; i < pathCopy.poses.size(); i++)
    {
        ROS_DEBUG_STREAM(i << ": " << pathCopy.poses[i]);
    }

    // Check if the path is the one we requested as a replan
    platform_motion_msgs::Knot initialKnot;
    initialKnot.pose = pathCopy.poses[0].pose;
    initialKnot.twist = odometry.twist.twist;

    // If we have not timed out waiting on a stitched path and we requested this path previously...
    if (((timestamp-start_time_wait_on_stitched_path) < ros::Duration(wait_on_stitched_path_duration)) && is_replan_ahead_iter_valid)
    {
        Eigen::Vector3d replanAheadPos;
        Eigen::Quaterniond replanAheadQuat;
        tf::pointMsgToEigen((*replan_ahead_iter).pose.position, replanAheadPos);
        tf::quaternionMsgToEigen((*replan_ahead_iter).pose.orientation, replanAheadQuat);

        Eigen::Vector3d newPathFirstPos;
        Eigen::Quaterniond newPathFirstQuat;
        tf::pointMsgToEigen(pathCopy.poses[0].pose.position, newPathFirstPos);
        tf::quaternionMsgToEigen(pathCopy.poses[0].pose.orientation, newPathFirstQuat);
        ROS_ERROR("Is replan ahead iter valid");
        ROS_ERROR_STREAM("newPathFirst: " << pathCopy.poses[0] << " replanAheadPos: " << (*replan_ahead_iter));

        if (((replanAheadPos - newPathFirstPos).squaredNorm() <= 0.0025) &&
            (replanAheadQuat.dot(newPathFirstQuat) > 0.9952)) // 0.9952 = cos(11.25/2.0 deg) to account for sbpl's 16 known angles
        {
            // Find the robot's current position in the stitched path
            auto currentKnotIter = stitched_path.knots.begin();
            for (; currentKnotIter != stitched_path.knots.end(); ++currentKnotIter)
            {
                if ((*currentKnotIter).header.seq == completed_knot_header.seq &&
                    fabs(((*currentKnotIter).header.stamp - completed_knot_header.stamp).toSec()) < 0.01)
                {
                    break;
                }
            }
            if (currentKnotIter == stitched_path.knots.end())
            {
                ROS_ERROR("Could not find current knot inside stitched path. Ignoring path request");
                return true;
            }

            Eigen::Vector3d currentPos;
            tf::pointMsgToEigen((*currentKnotIter).pose.position, currentPos);

            // Find the point in the stitched path that we want to stitch the new path to
            std::vector<platform_motion_msgs::Knot>::iterator lookAheadBufferKnotIter = stitched_path.knots.begin();
            for (auto prevPathIter = stitched_path.knots.begin(); prevPathIter != stitched_path.knots.end(); ++prevPathIter)
            {
                Eigen::Vector3d prevPathPos, prevPathPrevPos;
                Eigen::Quaterniond prevPathQuat, prevPathPrevQuat;
                tf::pointMsgToEigen((*prevPathIter).pose.position, prevPathPos);
                tf::quaternionMsgToEigen((*prevPathIter).pose.orientation, prevPathQuat);

                if (prevPathIter != stitched_path.knots.begin())
                {
                    tf::pointMsgToEigen((*(prevPathIter-1)).pose.position, prevPathPrevPos);
                    tf::quaternionMsgToEigen((*(prevPathIter-1)).pose.orientation, prevPathPrevQuat);
                }
                else
                {
                    continue;
                }

                bool isAheadEnoughInTime = ((*prevPathIter).header.stamp - completed_knot_header.stamp) > ros::Duration(replan_look_ahead_buffer_time);
                bool isAheadEnoughInDistance = (prevPathPos - currentPos).norm() >= (replan_look_ahead_time * maximum_linear_velocity);
                bool isPosApproxPrev = (prevPathPos- prevPathPrevPos).squaredNorm() < 0.0022;
                bool isQuatApproxPrev = prevPathQuat.dot(prevPathPrevQuat) > 0.9952; // 0.9952 = cos(11.25/2.0 deg)
                ROS_DEBUG_STREAM("BufferPose - isAheadEnoughInTime: " << isAheadEnoughInTime << "isAheadEnoughInDistance: " << isAheadEnoughInDistance << ", IsPosApproxPrev: " << isPosApproxPrev << ", IsQuatApproxPrev: " << isQuatApproxPrev);
                if (isAheadEnoughInTime &&  isAheadEnoughInDistance && !isPosApproxPrev && isQuatApproxPrev)
                {
                    ROS_ERROR("timestamp for first lookahead buffer pose ok!");
                    lookAheadBufferKnotIter = prevPathIter;
                    geometry_msgs::PoseStamped bufferStamped;
                    bufferStamped.pose = (*lookAheadBufferKnotIter).pose;
                    bufferStamped.header = (*lookAheadBufferKnotIter).header;
                    buffer_point_publisher.publish(bufferStamped);
                    geometry_msgs::PoseStamped stitchedStamped;
                    stitchedStamped.pose = (*replan_ahead_iter).pose;
                    stitchedStamped.header = (*replan_ahead_iter).header;
                    desired_stitch_point_publisher.publish(stitchedStamped);
                    break;
                }
            }
            if(lookAheadBufferKnotIter == stitched_path.knots.end() || lookAheadBufferKnotIter > replan_ahead_iter)
            {
                ROS_WARN("Planner got behind servos, cannot stitch new plan");
                is_replan_ahead_iter_valid = false;
                return true;
            }

            std::vector<geometry_msgs::PoseStamped> insertPoses;
            auto insertKnotIter = lookAheadBufferKnotIter;
            for (; insertKnotIter < replan_ahead_iter; ++insertKnotIter)
            {
                geometry_msgs::PoseStamped insertPose;
                insertPose.pose = (*insertKnotIter).pose;
                insertPoses.push_back(insertPose);
                ROS_DEBUG_STREAM("adding pose to front of replanned path: " << insertPose);
            }

            // Don't add the stitch point itself if it overlaps with the new path's first pose nearly identically
            if (replanAheadQuat.dot(newPathFirstQuat) <= 0.9999) // 0.9999 = cos(PI/180.0/2.0 rad)
            {
                geometry_msgs::PoseStamped insertPose;
                insertPose.pose = (*replan_ahead_iter).pose;
                insertPoses.push_back(insertPose);
                ROS_DEBUG("Adding stitch point because yaw is different enough. This should yield a turn-in-place");
            }

            if (insertPoses.size() > 0)
            {
                ROS_ERROR("Removing turn-in-place intermediate points");
                bool lastPointWasTurnInPlace = false;
                for (unsigned int i = 0; i < insertPoses.size()-1; ++i)
                {
                    Eigen::Vector3d currentPoint;
                    Eigen::Vector3d nextPoint;
                    tf::pointMsgToEigen(insertPoses[i].pose.position, currentPoint);
                    tf::pointMsgToEigen(insertPoses[i+1].pose.position, nextPoint);
                    if((currentPoint - nextPoint).squaredNorm() < 0.00001)
                    {
                        if(lastPointWasTurnInPlace)
                        {
                            posesToRemove.push_back(i);
                        }
                        lastPointWasTurnInPlace = true;
                    }
                    else
                    {
                        lastPointWasTurnInPlace = false;
                    }
                }
            }

            ROS_ERROR("pathCopy.poses size before prepend: %ld", pathCopy.poses.size());
            pathCopy.poses.insert(pathCopy.poses.begin(), insertPoses.begin(), insertPoses.end());
            ROS_ERROR("pathCopy.poses size after prepend: %ld", pathCopy.poses.size());
            ROS_ERROR("inserted %ld poses at front of replan path", insertPoses.size());

            // Set the initial knot to the first one in the spliced path
            initialKnot = (*lookAheadBufferKnotIter);

            // Set the timestamp to the time of the first point of
            // new path we are computing and also the time we want
            // platform_motion to splice this path into whatever
            // the robot is doing
            timestamp = (*lookAheadBufferKnotIter).header.stamp;
            ROS_ERROR_STREAM("timestamp now " << timestamp);
        }
        else
        {
            ROS_ERROR("We appear to have gotten a new plan while executing an old one. We're going to ignore it for now and pick it up in the next replan.");
            return true;
        }
    }
    else if (!isGoalReached() && (stitched_path.knots.size()>0) && (timestamp - stitched_path.knots.back().header.stamp) < ros::Duration(delta_time_after_goal_drop_path) )
    {
        // TODO - Send PVT to zero twist path
        return false;
    }

    // keep track of special case turn in place motions so we can remove the intermediate ones
    bool lastPointWasTurnInPlace = false;
    unsigned int i = 0;
    if (posesToRemove.size() > 0)
    {
        i = posesToRemove.back();
    }
    for (unsigned int i = 0; i < pathCopy.poses.size()-1; ++i)
    {
        Eigen::Vector3d currentPoint;
        Eigen::Vector3d nextPoint;
        tf::pointMsgToEigen(pathCopy.poses[i].pose.position, currentPoint);
        tf::pointMsgToEigen(pathCopy.poses[i+1].pose.position, nextPoint);
        if((currentPoint - nextPoint).squaredNorm() < 0.00001)
        {
            if(lastPointWasTurnInPlace)
            {
                posesToRemove.push_back(i);
            }
            lastPointWasTurnInPlace = true;
        }
        else
        {
            lastPointWasTurnInPlace = false;
        }
    }
    for(auto iter = posesToRemove.rbegin(); iter != posesToRemove.rend(); ++iter)
    {
        pathCopy.poses.erase(pathCopy.poses.begin()+(*iter));
    }

    ROS_DEBUG("pathCopy post erasing:");

    for(unsigned int i = 0; i < pathCopy.poses.size(); i++)
    {
        ROS_DEBUG_STREAM(i << ": " << pathCopy.poses[i]);
    }

    // What direction is the robot going and what is the linear/angular vel?
    platform_motion_msgs::Path path_msg;
    path_msg.knots.resize(pathCopy.poses.size());
    path_msg.header.stamp = Time::now();
    path_msg.header.seq = 0;
    path_msg.header.frame_id = "map";

    visualization_msgs::MarkerArray visualizationMarkerArray;
    visualizationMarkerArray.markers.resize(3*(pathCopy.poses.size()-1));

    // Populate the first entry with the current kinematic data.
    if (pathCopy.poses.size() > 0)
    {
        path_msg.knots[0] = initialKnot;
        path_msg.knots[0].header.seq = 0;
        path_msg.knots[0].header.stamp = timestamp;
        path_msg.knots[0].header.frame_id = "map";
    }

    // Store minimum time stamps for all poses in the path ensuring that there is enough
    // time to achieve each one.  For our robot, this time is dominated by the rate at
    // which the wheels can rotate to achieve the desired wheel angle
    Eigen::Vector3d pathLinearVelocity;
    tf::vectorMsgToEigen(odometry.twist.twist.linear, pathLinearVelocity);
    double pathVelocityMagnitude = pathLinearVelocity.norm();
    double initialPathVelocity = pathVelocityMagnitude;
    BezierCubicSpline<Eigen::Vector3d>* splines = new BezierCubicSpline<Eigen::Vector3d>[pathCopy.poses.size()-1];

    double totalDistance = 0.0;

    for (unsigned int i = 0; i < pathCopy.poses.size()-1; ++i)
    {
        bool success = this->FitCubicSpline(pathCopy, i, pathVelocityMagnitude, maximum_linear_velocity, splines[i]);
        if (!success)
        {
            ROS_DEBUG("Failed to fit a cubic spline to the path. FIX ME!");
        }

        double distanceTraveled = splines[i].ComputeArcLength();

        totalDistance += distanceTraveled;

        if(distanceTraveled < 0.0)
        {
            ROS_DEBUG("distanceTraveled is %f for point %d!!!", distanceTraveled, i);
        }

        // Account for acceleration rate and compute the actual velocity at the next point
        double nextVelocityMagnitude = sqrt(initialPathVelocity*initialPathVelocity + 2.00*linear_acceleration*totalDistance);
        nextVelocityMagnitude = min(nextVelocityMagnitude, maximum_linear_velocity);

        if(distanceTraveled < 0.001)
        {
            // if we barely move, we are probably in a turn in place and want to be at zero velocity!
            nextVelocityMagnitude = 0.0;
            pathVelocityMagnitude = 0.0;
            totalDistance = 0.0;
        }

        // Compute the minimum about of time necessary for the robot to traverse the
        // spline with starting and ending velocities.  This accounts for the specific
        // kinematic constraints, such as the wheel angle rotation rate limit
        //double minimumPathTime = this->ComputeMinimumPathTime(splines[i], pathVelocityMagnitude, nextVelocityMagnitude);
        double minimumPathTime = this->ComputeMinimumPathTime(pathCopy, i);
        ROS_DEBUG_STREAM("minimumPathTime: " << minimumPathTime);
        if (nextVelocityMagnitude < maximum_linear_velocity)
        {
            ROS_DEBUG_STREAM("nextVelocityMagnitude < maximum_linear_velocity. minimumPathTime: " << minimumPathTime);
            minimumPathTime = max((nextVelocityMagnitude-pathVelocityMagnitude)/linear_acceleration, minimumPathTime);
        }
        else
        {
            ROS_DEBUG_STREAM("nextVelocityMagnitude >= maximum_linear_velocity. minimumPathTime: " << minimumPathTime);
            minimumPathTime = max(distanceTraveled/maximum_linear_velocity, minimumPathTime);
        }

        // Update the timestamp
        timestamp += ros::Duration(minimumPathTime);
        ROS_DEBUG_STREAM("timestamp now " << timestamp);

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
        Eigen::Quaterniond nextQuaternion(pathCopy.poses[i+1].pose.orientation.w,
                                          pathCopy.poses[i+1].pose.orientation.x,
                                          pathCopy.poses[i+1].pose.orientation.y,
                                          pathCopy.poses[i+1].pose.orientation.z);
        Eigen::Vector3d nextVelocityVector = nextQuaternion * Eigen::Vector3d(1.00, 0.00, 0.00);
        nextVelocityVector *= nextVelocityMagnitude;

        // Compute the next angular velocity
        //double curvature = splines[i].ComputeCurvature(1.00);
        //double angularVelocity = nextVelocityMagnitude * curvature;
        double curvature = ComputeCurvature(pathCopy, i);
        double startYaw = yawFromMsgQuat(pathCopy.poses[i].pose.orientation);
        double endYaw = yawFromMsgQuat(pathCopy.poses[i+1].pose.orientation);
        if ((endYaw - startYaw) > M_PI)
        {
            startYaw += 2.0*M_PI;
        }
        else if ((endYaw - startYaw) < -M_PI)
        {
            startYaw -= 2.0*M_PI;
        }
        double angularVelocity = 0.00;
        if (minimumPathTime > 0.00)
        {
            angularVelocity = (endYaw - startYaw)/minimumPathTime;
        }
        if (fabs(curvature*nextVelocityMagnitude) > fabs(angularVelocity))
        {
            angularVelocity = curvature*nextVelocityMagnitude;
        }
        if (distanceTraveled < 0.001)
        {
            angularVelocity = 0.0; // Special case for turn-in-place
        }
        ROS_DEBUG("curvature: %f. angular velocity: %f", curvature, angularVelocity);

        // Update path velocity
        pathVelocityMagnitude = nextVelocityMagnitude;

        // Fill the outbound message data
        path_msg.knots[i+1].header.seq = i+1;
        path_msg.knots[i+1].header.stamp = timestamp;
        path_msg.knots[i+1].header.frame_id = "map";
        path_msg.knots[i+1].pose = pathCopy.poses[i+1].pose;
        ROS_DEBUG_STREAM("timestamp now " << timestamp);
        ROS_DEBUG_STREAM("timestamp in knot itself is " << path_msg.knots[i+1].header.stamp);

        path_msg.knots[i+1].twist.linear.x = nextVelocityVector.x();
        path_msg.knots[i+1].twist.linear.y = nextVelocityVector.y();
        path_msg.knots[i+1].twist.linear.z = 0.0;
        path_msg.knots[i+1].twist.angular.x = 0.0;
        path_msg.knots[i+1].twist.angular.y = 0.0;
        path_msg.knots[i+1].twist.angular.z = angularVelocity;
    }

    // Set the velocity at the last point to be zero
    path_msg.knots[pathCopy.poses.size()-1].twist.linear.x = 0.00;
    path_msg.knots[pathCopy.poses.size()-1].twist.linear.y = 0.00;
    path_msg.knots[pathCopy.poses.size()-1].twist.linear.z = 0.00;
    path_msg.knots[pathCopy.poses.size()-1].twist.angular.z = 0.00;

    ROS_DEBUG("\n\npath_msg.knots:");
    for(unsigned int i = 0; i < path_msg.knots.size(); i++)
    {
        ROS_DEBUG_STREAM(i << ": " << path_msg.knots[i]);
    }
    
    // Compute and store smooth decelerations
    for (unsigned int i = pathCopy.poses.size()-1; i > 0; --i)
    {
        Eigen::Vector3d currentVelocityVec;
        Eigen::Vector3d previousVelocityVec;
        tf::vectorMsgToEigen(path_msg.knots[i].twist.linear, currentVelocityVec);
        tf::vectorMsgToEigen(path_msg.knots[i-1].twist.linear, previousVelocityVec);
        double currentVelocityMagnitude = currentVelocityVec.norm();
        double previousVelocityMagnitude = previousVelocityVec.norm();
        double distanceTraveled = splines[i-1].ComputeArcLength();
        double expectedVelocityMagnitude = sqrt(currentVelocityMagnitude*currentVelocityMagnitude + 2.00*linear_acceleration*distanceTraveled);
        if (previousVelocityMagnitude > (expectedVelocityMagnitude+0.005))
        {
            double previousVelocityScale = expectedVelocityMagnitude/previousVelocityMagnitude;
            if (previousVelocityScale < 0.01)
            {
                previousVelocityScale = 0.0; // Hack to handle numerical precision problems :(
            }
            path_msg.knots[i-1].twist.linear.x *= previousVelocityScale;
            path_msg.knots[i-1].twist.linear.y *= previousVelocityScale;
            path_msg.knots[i-1].twist.linear.z = 0.00;
            path_msg.knots[i-1].twist.angular.z *= previousVelocityScale;
        }

    }

    for(unsigned int i = 0; i < path_msg.knots.size()-1; i++)
    {
        // modify the time in i+1 to match the actual velocity between points i and i+1
        Eigen::Vector3d currentVelocityVec;
        Eigen::Vector3d nextVelocityVec;
        tf::vectorMsgToEigen(path_msg.knots[i].twist.linear, currentVelocityVec);
        tf::vectorMsgToEigen(path_msg.knots[i+1].twist.linear, nextVelocityVec);
        double currentVelocityMagnitude = currentVelocityVec.norm();
        double nextVelocityMagnitude = nextVelocityVec.norm();
        double distanceTraveled = splines[i].ComputeArcLength();

        if((nextVelocityMagnitude+currentVelocityMagnitude) > 0.0001)
        {
            path_msg.knots[i+1].header.stamp = path_msg.knots[i].header.stamp +
                ros::Duration(distanceTraveled*2/(nextVelocityMagnitude+currentVelocityMagnitude));
        }
        else
        {
            path_msg.knots[i+1].header.stamp = path_msg.knots[i].header.stamp;
        }
    }

    ROS_DEBUG("\n\npath_msg.knots post decel:");
    for(unsigned int i = 0; i < path_msg.knots.size(); i++)
    {
        ROS_DEBUG_STREAM(i << ": " << path_msg.knots[i]);
    }
    

    // now it is time to add the turn in place segments!
    // loop back over the path, and add a bunch of magical turn in place segments!
    // keep track of accumulated turn segment time offset to we can make sure all the
    // timestamps work out still..
    ros::Duration totalTurnTime(0);
    unsigned int totalNumTurnKnots = 0;
    for(unsigned int i = 0; i < path_msg.knots.size()-1; i++)
    {
        // add the turn time to the time stamp to keep all the timestamps accurate
        path_msg.knots[i].header.stamp += totalTurnTime;
        path_msg.knots[i].header.seq += totalNumTurnKnots;

        Eigen::Vector3d currentPoint, nextPoint;
        tf::pointMsgToEigen(path_msg.knots[i].pose.position, currentPoint);
        tf::pointMsgToEigen(path_msg.knots[i+1].pose.position, nextPoint);
        if((currentPoint - nextPoint).squaredNorm() < 0.00001)
        {
            ros::Duration turnTime;
            auto turnKnots = computeTurnInPlace(
                    path_msg.knots[i],
                    path_msg.knots[i+1],
                    turnTime,
                    sternPodVector
            );

            if (turnKnots.size() == 0)
            {
                ROS_ERROR("Turn in place was between the same angles. Skipping...");
                continue;
            }

            // drop the last knot to make stitching a little easier...
            turnKnots.pop_back();

            // keep track of the total time these turns are adding so we can offset timestamps
            // accordingly
            totalTurnTime += turnTime;

            // add the new knots to the path remembering that insert inserts BEFORE the given element!
            path_msg.knots.insert(path_msg.knots.begin()+i+1, turnKnots.begin(), turnKnots.end());

            ROS_ERROR("Adding turn-in-place at index %d with %d segments", i, (int)(turnKnots.size()));

            // since the turn in place path will probably pass the test in this if statement, we
            // need to advance i by enough places so the new points will be skipped and we'll end up
            // at what was i+1 before we did the above insert.
            i+=turnKnots.size();
            totalNumTurnKnots += turnKnots.size();

            ROS_ERROR("After adding turn in place, jumping to path index %d", (int)i+1);
        }
    }

    ROS_DEBUG("\n\npath_msg.knots post turn in place inflation:");
    for(unsigned int i = 0; i < path_msg.knots.size(); i++)
    {
        ROS_DEBUG_STREAM(i << ": " << path_msg.knots[i]);
    }

    // the last point is never examined in the loop, so add the turn duration to it here.
    path_msg.knots.back().header.stamp += totalTurnTime;
    path_msg.knots.back().header.seq += totalNumTurnKnots;

    // Publish path
    smooth_path_publisher.publish(path_msg);
    visualization_publisher.publish(visualizationMarkerArray);

    is_replan_ahead_iter_valid = false;
    is_waiting_on_stitched_path = true;
    is_goal_reached = false;
    have_goal = true;
    start_time_wait_on_stitched_path = Time::now();

    return true;
}

void TheSmoothPlanner::setOdometry(const nav_msgs::Odometry& odometry)
{
    ROS_DEBUG("RECEIVED ODOMETRY");
    this->odometry = odometry;
    return;
}

void TheSmoothPlanner::setCompletedKnot(const std_msgs::Header& completedKnotHeader)
{
    this->completed_knot_header = completedKnotHeader;

    // Invalide the replan ahead iter if we have completed the knot past the buffer time
    if ( is_replan_ahead_iter_valid &&
         (completedKnotHeader.stamp >= ((*replan_ahead_iter).header.stamp - ros::Duration(replan_look_ahead_buffer_time))) ||
         (completedKnotHeader.seq == stitched_path.knots.back().header.seq && completedKnotHeader.stamp == stitched_path.knots.back().header.stamp) )
    {
        this->is_replan_ahead_iter_valid = false;
    }

    this->is_goal_reached = (stitched_path.knots.size() > 0 &&
                            completed_knot_header.seq == stitched_path.knots.back().header.seq &&
                            fabs((completed_knot_header.stamp - stitched_path.knots.back().header.stamp).toSec()) < 0.01);
    if( this->is_goal_reached )
    {
        have_goal = false;
    }
    ROS_ERROR_STREAM("RECEIVED COMPLETED KNOT, reached: " << this->is_goal_reached);

    // check to see if it has been too long since we've heard form the planner.
    if(((Time::now() - timeSinceLastPlanFromPlanner) > plannerTimeoutDuration) &&
        (stitched_path.knots.size() > 0))
    {
        ROS_ERROR_STREAM("Planner not responding with replans! this is bad! It has been " << (Time::now() - timeSinceLastPlanFromPlanner) << " since we heard form the planner!");

        ROS_ERROR("Planning to halt asap!");
        // create a pathological path that will cause platform motion to stop
        // everything and plan to a hard zero velocity.
        platform_motion_msgs::Path path_msg;
        path_msg.header.stamp = Time::now();
        path_msg.header.seq = 0;
        path_msg.header.frame_id = "map";
        ROS_ERROR("Publishing crazy plan to cause stop!");
        smooth_path_publisher.publish(path_msg);
        is_waiting_on_stitched_path = true;

        // reset the time so we don't have problems later
        timeSinceLastPlanFromPlanner = Time::now();
    }
}

void TheSmoothPlanner::setMaximumVelocity(const std_msgs::Float64::ConstPtr velocity)
{
    this->maximum_linear_velocity = velocity->data;
}

void TheSmoothPlanner::setStitchedPath(const platform_motion_msgs::Path& stitchedPath)
{
    this->stitched_path = stitchedPath;
    this->replan_ahead_iter = this->stitched_path.knots.begin();
    this->is_replan_ahead_iter_valid = false;
    this->is_waiting_on_stitched_path = false;
    ROS_ERROR("Received stitched path of size %ld", stitchedPath.knots.size());
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

double TheSmoothPlanner::ComputeMinimumPathTime(const nav_msgs::Path& path, unsigned int i)
{
    const double phiDotMax = M_PI/4.00;

    double curvature = this->ComputeCurvature(path, i);
    double deltaCurvature = this->ComputeCurvature(path, i+1) - curvature;
    double deltaPhi = -deltaCurvature*sternPodVector(0)*cos( -atan(curvature*sternPodVector(0))*atan(curvature*sternPodVector(0)) );
    // anything more curvy than 1.8 is turn in place...
    if (fabs(deltaCurvature) > 1.8 ) // 1.8 is approximately equal to infinity for small values of infinity
    {
        deltaPhi = M_PI/2.00;
    }
    double minimumDeltaTime = fabs(deltaPhi)/phiDotMax;
}

double TheSmoothPlanner::ComputeMinimumPathTime(const BezierCubicSpline<Eigen::Vector3d>& spline,
                                                double initialVelocity,
                                                double finalVelocity)
{
    double distanceTraveled = spline.ComputeArcLength(0.01);
    double averageVelocity = (initialVelocity + finalVelocity)/2.00;
    if((distanceTraveled < 0.0001) || (averageVelocity < 0.0001))
    {
        ROS_ERROR("Error distance traveled %f, averageVelocity %f. Setting minimum path time to 0!", distanceTraveled, averageVelocity);
        return 0.0;
    }
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
    //  requiredDeltaSternAngle = deltaCurvature / (nextCircleCurvature*nextCircleCurvature) * (1.00/sternPodVector(0)) * sineSternAngle * sineSternAngle;
    //}
    //else
    //{
    //  requiredDeltaSternAngle = 0.0;
    //}
    //double minimumDeltaTime = fabs(requiredDeltaSternAngle) / maximum_slew_radians_per_second;

    return minimumDeltaTime;
}

double TheSmoothPlanner::ComputeCurvature(const nav_msgs::Path& path, const unsigned int i)
{
    unsigned int im1 = (i >= 1) ? i-1 : 0;
    unsigned int ip1 = (i < path.poses.size()-1) ? i+1 : path.poses.size()-1;
    double im1iLength = (i >= 1) ? sqrt(pow(path.poses[i].pose.position.x-path.poses[im1].pose.position.x,2) + pow(path.poses[i].pose.position.y-path.poses[im1].pose.position.y,2)) : 0.0;
    double iip1Length = (i < path.poses.size()-1) ? sqrt(pow(path.poses[ip1].pose.position.x-path.poses[i].pose.position.x,2) + pow(path.poses[ip1].pose.position.y-path.poses[i].pose.position.y,2)) : 0.0;
    double deltaLength = im1iLength + iip1Length;

    double curvature = 0.00;
    double deltaX = 0.00;
    double deltaY = 0.00;
    if (deltaLength > 0.01)
    {
        deltaX = (path.poses[ip1].pose.position.x - path.poses[im1].pose.position.x)/deltaLength;
        deltaY = (path.poses[ip1].pose.position.y - path.poses[im1].pose.position.y)/deltaLength;

        double deltaSquaredX = (path.poses[ip1].pose.position.x - 2.0*path.poses[i].pose.position.x + path.poses[im1].pose.position.x)/deltaLength;
        double deltaSquaredY = (path.poses[ip1].pose.position.y - 2.0*path.poses[i].pose.position.y + path.poses[im1].pose.position.y)/deltaLength;
        if (deltaX != 0.00 || deltaY != 0.00)
        {
            curvature = (deltaX*deltaSquaredY - deltaY*deltaSquaredX)/pow(deltaX*deltaX + deltaY*deltaY, 3.0/2.0);
        }
    }

    return curvature;
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
