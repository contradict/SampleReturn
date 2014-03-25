#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <platform_motion_msgs/Path.h>
#include <platform_motion_msgs/ScaryTestMode.h>

namespace platform_motion {

class TestPath {
    public:
    TestPath(void);
    void init(void);

    private:
        ros::Subscriber scary_test_mode_sub_;
        ros::Publisher path_pub_;

        tf::TransformListener listener;

        // scary pvt test mode enable. only do this on blocks!!!
        void scaryTestModeCallback(const platform_motion_msgs::ScaryTestMode::ConstPtr msg);
        std::list<platform_motion_msgs::Knot> computeCirclePath(double dt, double R, double a, double vmax);
        std::list<platform_motion_msgs::Knot> computeStraightPath(double dt, double tconst, double a, double vmax);
        std::list<platform_motion_msgs::Knot> computeFigureEight(int numPoints, double amplitude, double omega, double acceleration);
        std::list<platform_motion_msgs::Knot> computeScaryPath(ros::Time time, int which);

};

TestPath::TestPath()
{
}

void TestPath::init(void)
{
    ros::NodeHandle nh;
    path_pub_ = nh.advertise<platform_motion_msgs::Path>("scary_path", 1);
    scary_test_mode_sub_ = nh.subscribe(
            "scary_test_mode",
            1,
            &TestPath::scaryTestModeCallback,
            this
    );
}

void TestPath::scaryTestModeCallback(const platform_motion_msgs::ScaryTestMode::ConstPtr msg)
{
    platform_motion_msgs::Path path;
    // keep track of when we started.
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    std::list<platform_motion_msgs::Knot> knots = computeScaryPath(path.header.stamp, msg->which);
    path.knots.insert(path.knots.begin(), knots.begin(), knots.end() );

    path_pub_.publish(path);
}

std::list<platform_motion_msgs::Knot> TestPath::computeStraightPath(double dt, double tconst, double a, double vmax)
{
    std::list<platform_motion_msgs::Knot> retval;

    platform_motion_msgs::Knot knot;

    ros::Duration step = ros::Duration(dt);

    while(knot.twist.linear.x<vmax)
    {
        retval.push_back(knot);
        knot.header.stamp += step;
        knot.header.seq += 1;
        double t = (knot.header.stamp).toSec();
        knot.twist.linear.x  = a*t;
        knot.pose.position.x = dt*a*t*t;
    }
    knot = retval.back();
    knot.twist.linear.x = vmax;
    for(double t=0;t<tconst;t+=dt) {
        knot.header.stamp += step;
        knot.header.seq += 1;
        knot.pose.position.x += vmax*dt;
        retval.push_back(knot);
    }
    ros::Time tdecel=knot.header.stamp;
    double xdecel=knot.pose.position.x;
    while(knot.twist.linear.x>0)
    {
        knot.header.stamp += step;
        knot.header.seq += 1;
        double t = (knot.header.stamp - tdecel).toSec();
        knot.twist.linear.x  = vmax - a*t;
        knot.pose.position.x = xdecel + (vmax*t - dt*a*t*t);
        if(knot.twist.linear.x>0) retval.push_back(knot);
    }
    knot = retval.back();
    double decelrest=knot.twist.linear.x/a;
    knot.header.stamp    += ros::Duration(decelrest);
    knot.pose.position.x += dt*a*decelrest*decelrest;
    knot.twist.linear.x   = 0;
    retval.push_back(knot);

    return retval;
}

std::list<platform_motion_msgs::Knot> TestPath::computeCirclePath(double dt, double R, double a, double vmax)
{
    std::list<platform_motion_msgs::Knot> retval;

    platform_motion_msgs::Knot knot;

    ros::Duration step = ros::Duration(dt);

    double s=0, v=0, t=0, theta=0;
    retval.push_back(knot);
    while(v<vmax)
    {
        knot.header.stamp += step;
        knot.header.seq += 1;
        t += dt;
        v = a*t;
        s = dt*a*t*t;
        theta = s/R;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,theta), knot.pose.orientation);
        knot.twist.angular.z = v/R;
        knot.pose.position.x = R*sin(theta);
        knot.twist.linear.x = v*cos(theta);
        knot.pose.position.y = R*(1-cos(theta));
        knot.twist.linear.y = v*sin(theta);
        retval.push_back(knot);
    }
    double acceltheta=theta;
    while(theta<(2*M_PI-acceltheta))
    {
        knot.header.stamp += step;
        knot.header.seq += 1;
        t += dt;
        s += v*dt;
        theta = s/R;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,theta), knot.pose.orientation);
        knot.twist.angular.z = v/R;
        knot.pose.position.x = R*sin(theta);
        knot.twist.linear.x = v*cos(theta);
        knot.pose.position.y = R*(1-cos(theta));
        knot.twist.linear.y = v*sin(theta);
        retval.push_back(knot);
    }
    t=0;
    double vdecel=v, sdecel=s;
    while(v>0)
    {
        knot.header.stamp += step;
        knot.header.seq += 1;
        t += dt;
        v  = vdecel-a*t;
        s  = sdecel+vdecel*t-dt*a*t*t;
        theta = s/R;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,theta), knot.pose.orientation);
        knot.twist.angular.z = v/R;
        knot.pose.position.x = R*sin(theta);
        knot.twist.linear.x = v*cos(theta);
        knot.pose.position.y = R*(1-cos(theta));
        knot.twist.linear.y = v*sin(theta);
        retval.push_back(knot);
    }
    return retval;
}

std::list<platform_motion_msgs::Knot> TestPath::computeFigureEight(int numPoints, double amplitude, double omega, double acceleration)
{
    std::list<platform_motion_msgs::Knot> retval;
    // compute the time step between each point
    ros::Duration timeStep = ros::Duration((2.0*M_PI/omega)/((double)numPoints));
    // this is based on the math in path2local.py. sorry the contradict-style variable names
    // declare lambdas for the loop.
    auto f = [] (double x) {return 2.0 * atan(x);};
    auto fprime = [] (double x) {return 2.0/(1.0 + pow(x, 2));};
    auto g = [] (double x, double y) {return (sqrt(pow(x, 2) + pow(y, 2)) - x) / y;};
    auto gprime = [] (double x, double y, double xdot, double ydot)
        {return (pow((pow(x, 2) + pow(y, 2)), -0.5) * (x * xdot + y * ydot) - xdot) / y -
            (sqrt(pow(x, 2) + pow(y,2)) -x) / pow(y, 2) * ydot;};

    // compute the first point so that we have a nice ramp up to the start of the
    // figure 8. start by computing the first point of the figure 8.
    double xDot0 = 2.0 * amplitude * omega;
    double yDot0 = 2.0 * amplitude * omega;
    double theta0 = f(g(xDot0, yDot0));
    double startDistance = 0.5 * (pow(xDot0, 2.0) + pow(yDot0, 2.0)) / acceleration;
    double startDuration = sqrt(pow(xDot0, 2.0) + pow(yDot0, 2.0)) / acceleration;
    platform_motion_msgs::Knot start;
    start.pose.position.x = startDistance * cos(theta0 + M_PI);
    start.pose.position.y = startDistance * sin(theta0 + M_PI);
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,theta0), start.pose.orientation);

    retval.push_back(start);

    // account for the start up ramp by adding some to the actual start time
    ros::Time actualStartTime = ros::Time(0) + ros::Duration(startDuration);

    platform_motion_msgs::Knot knot;

    int seq=1;
    double theta;
    // build the figure 8 part of the path
    for(int i = 0; i < numPoints; i++)
    {
        double t = (timeStep * i).toSec();
        knot.header.stamp = actualStartTime + timeStep * i;
        knot.header.seq = seq++;
        knot.pose.position.x = 2.0 * amplitude * sin(omega * t);
        knot.twist.linear.x = 2.0 * amplitude * omega * cos(omega * t);
        double xdDot = -2.0 * amplitude * omega * omega * sin(omega * t);
        knot.pose.position.y = amplitude * sin(2.0 * omega * t);
        knot.twist.linear.y = 2.0 * amplitude * omega * cos(2.0 * omega * t);
        double ydDot = -4.0 * amplitude * omega * omega * sin(2.0 * omega * t);
        theta = f(g(knot.twist.linear.x, knot.twist.linear.y));
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,theta), knot.pose.orientation);
        knot.twist.angular.z = fprime(g(knot.twist.linear.x, knot.twist.linear.y)) *
            gprime(knot.twist.linear.x, knot.twist.linear.y, xdDot, ydDot);

        retval.push_back(knot);
    }

    // now add a ramp down to zero at the end.
    platform_motion_msgs::Knot end;
    double endDistance = 0.5 * (pow(retval.back().twist.linear.x, 2.0) + pow(retval.back().twist.linear.y, 2.0))
        / acceleration;
    double endDuration = sqrt(pow(retval.back().twist.linear.x, 2.0) + pow(retval.back().twist.linear.y, 2.0)) /
        acceleration;

    end.header.stamp = retval.back().header.stamp + ros::Duration(endDuration);
    end.header.seq = seq++;
    end.twist.linear.x = 0.0;
    end.twist.linear.y = 0.0;
    end.twist.angular.z = 0.0;
    end.pose.position.x = endDistance * cos(theta) + retval.back().pose.position.x;
    end.pose.position.y = endDistance * sin(theta) + retval.back().pose.position.y;
    end.pose.orientation = retval.back().pose.orientation;

    retval.push_back(end);

    return retval;
}

std::list<platform_motion_msgs::Knot> TestPath::computeScaryPath(ros::Time time, int which)
{
    std::list<platform_motion_msgs::Knot> retval;

    platform_motion_msgs::Knot knot;
    knot.header.stamp = time;
    knot.header.seq = 0;
    knot.header.frame_id = "map";
    knot.pose.position.x=0;
    knot.pose.position.y=0;
    knot.pose.position.z=0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,0), knot.pose.orientation);
    knot.twist.linear.x=0;
    knot.twist.linear.y=0;
    knot.twist.linear.z=0;
    knot.twist.angular.x=0;
    knot.twist.angular.y=0;
    knot.twist.angular.z=0;

    double dt=0.5;
    double R=3.0;
    double a=0.1;
    double vmax=1.0;
    double tconst=5.0;
    int numPoints=250;
    double amplitude=3.0;
    double omega=0.1;

    switch(which) {
        case platform_motion_msgs::ScaryTestMode::Circle:
            retval=computeCirclePath(dt, R,  a, vmax);
            break;
        case platform_motion_msgs::ScaryTestMode::Straight:
            retval=computeStraightPath(dt, tconst, a, vmax);
            break;
        case platform_motion_msgs::ScaryTestMode::FigureEight:
            retval=computeFigureEight(numPoints, amplitude, omega, a);
            break;
        default:
            ROS_ERROR("Unknown path %d", which);
            return retval;
    }

    // debugging. should not be pushed commented in!
    //int count=0;
    //for(auto i : retval)
    //{
    //    ROS_DEBUG("count: %d, time: %f, x: %f, y: %f, theta: %f, xDot: %f, yDot: %f, thetaDot: %f",
    //           count, i.time.toSec(), i.x, i.y, i.theta, i.xDot, i.yDot, i.thetaDot
    //    );
    //    count++;
    //}
    return retval;

}

} // namespace platform_motion

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TestPath");
    platform_motion::TestPath p;
    p.init();
    ROS_INFO("spin");
    ros::spin();
}

