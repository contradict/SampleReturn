#include <limits>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <platform_motion_msgs/Path.h>
#include <platform_motion_msgs/ScaryTestMode.h>

namespace platform_motion {

class TestPath {
    public:
    TestPath(void);
    void init(void);

    private:
        const double wheelbase_;
        const double acceleration_;
        const double maxv_;
        const double alpha_;
        const double maxomega_;
        const double maxphidot_;
        const double dt_;
        bool have_path_;

        ros::Subscriber scary_test_mode_sub_;
        ros::Subscriber stitched_path_sub_;
        ros::Publisher path_pub_;

        tf::TransformListener listener;

        // scary pvt test mode enable. only do this on blocks!!!
        void scaryTestModeCallback(const platform_motion_msgs::ScaryTestMode::ConstPtr msg);
        void sendPath(int which, bool absolute);
        void stitchedPathCallback(const platform_motion_msgs::Path::ConstPtr msg);
        std::list<platform_motion_msgs::Knot> computeCirclePath(double R);
        std::list<platform_motion_msgs::Knot> computeStraightPath(double tconst);
        std::list<platform_motion_msgs::Knot> computeFigureEight(int numPoints, double amplitude, double omega);
        std::list<platform_motion_msgs::Knot> computeScaryPath(int which);

        bool waitForStitched(std::chrono::seconds timeout);
        void testMultipleSegments();

        std::vector< platform_motion_msgs::Knot > stitched_knots_;
        std::mutex knot_mutex_;
        std::condition_variable knot_condition_;
};

TestPath::TestPath() :
    wheelbase_(1.125),
    acceleration_(0.1),
    maxv_(1.5),
    alpha_(0.13),
    maxomega_(2.0),
    maxphidot_(M_PI/2),
    dt_(0.5),
    have_path_(false)
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

    stitched_path_sub_ = nh.subscribe(
            "stitched_path",
            1,
            &TestPath::stitchedPathCallback,
            this
            );
}

void TestPath::scaryTestModeCallback(const platform_motion_msgs::ScaryTestMode::ConstPtr msg)
{
    platform_motion_msgs::Path path;

    std::list<platform_motion_msgs::Knot> knots = computeScaryPath(msg->which);
    if( knots.size() > 0 )
    {
        // adjust times so first timestamp is 0
        // to ensure relative motion
        ros::Duration first=knots.front().header.stamp-ros::Time(0);
        if( msg->absolute )
        {
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "map";
            tf::StampedTransform initial;
            ros::Time most_recent(0);
            try {
                listener.lookupTransform("map", "base_link", most_recent, initial );
            } catch( tf::TransformException ex) {
                ROS_ERROR("Error looking up %s: %s", "current robot pose", ex.what());
                return;
            }
            Eigen::Quaterniond rotation;
            Eigen::Vector3d translation, velocity, omega;
            tf::quaternionTFToEigen( initial.getRotation(), rotation );
            tf::vectorTFToEigen( initial.getOrigin(), translation );
            std::transform( knots.begin(), knots.end(), knots.begin(),
                    [rotation, translation]( platform_motion_msgs::Knot k )
                    {
                    Eigen::Quaterniond orientation;
                    Eigen::Vector3d position, kvelocity, komega;

                    tf::quaternionMsgToEigen( k.pose.orientation, orientation );
                    tf::pointMsgToEigen(k.pose.position, position );
                    tf::vectorMsgToEigen( k.twist.linear, kvelocity );
                    tf::vectorMsgToEigen( k.twist.angular, komega );

                    orientation = rotation*orientation;
                    position = translation + rotation*position;
                    komega = rotation*komega;
                    kvelocity = rotation*kvelocity;

                    tf::quaternionEigenToMsg( orientation, k.pose.orientation );
                    tf::pointEigenToMsg( position, k.pose.position );
                    tf::vectorEigenToMsg( kvelocity, k.twist.linear );
                    tf::vectorEigenToMsg( komega, k.twist.angular );

                    return k;
                    });
        }
        else
        {
            path.header.stamp = ros::Time(0);
            path.header.frame_id = "base_link";
        }
        path.knots.insert(path.knots.begin(), knots.begin(), knots.end() );

        ROS_INFO( "Publish path." );
        path_pub_.publish(path);
    }
}

void TestPath::stitchedPathCallback(const platform_motion_msgs::Path::ConstPtr msg)
{
    ROS_INFO( "Got stitched path " );
    std::unique_lock<std::mutex> lock(knot_mutex_);
    stitched_knots_.clear();
    stitched_knots_.insert(stitched_knots_.end(), msg->knots.begin(), msg->knots.end());
    have_path_ = true;
    knot_condition_.notify_one();
}

bool TestPath::waitForStitched(std::chrono::seconds timeout)
{
    have_path_ = false;
    std::unique_lock<std::mutex> lock(knot_mutex_);
    knot_condition_.wait_for( lock, timeout, [this](){return this->have_path_;} );
    return have_path_;
}

std::list<platform_motion_msgs::Knot> leftHook( void )
{
    std::list<platform_motion_msgs::Knot> seg;
    platform_motion_msgs::Knot k;
    // set identity
    k.pose.orientation.w=1.0;
    seg.push_back(k);

    double Vmax=1.0;
    double L=5.0;
    // Accel to Vmax traveling a distance L/2
    k.pose.position.x = L/2.;
    k.twist.linear.x = Vmax;
    k.header.stamp += ros::Duration(5.0);
    seg.push_back(k);
    // Travel L/2 at Vmax
    k.pose.position.x = L;
    k.twist.linear.x = Vmax;
    k.header.stamp += ros::Duration((L/2.)/Vmax);
    seg.push_back(k);
    // Begin a left arc of radius R, decelerating to Vmax/4.0
    // at 1/4 circle
    double R=2.0;
    k.pose.position.x += R*cos(-M_PI_4);
    k.pose.position.y += R*(1.0+sin(-M_PI_4));
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw( M_PI_4 ), k.pose.orientation);
    k.twist.linear.x = -(Vmax/4.)*sin(-M_PI_4);
    k.twist.linear.y =  (Vmax/4.)*cos(-M_PI_4);
    k.twist.angular.z = (Vmax/4.)/R;
    k.header.stamp += ros::Duration(2.0*M_PI*R*0.25*2.0/((Vmax/2.)+Vmax));
    seg.push_back(k);
    // Finish left arc decelerating to 0.1m/s
    k.pose.position.x += R*(cos(0)       -  cos(-M_PI_4));
    k.pose.position.y += R*((1.0+sin(0)) - (1.0+sin(-M_PI_4)));
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw( M_PI_2 ), k.pose.orientation);
    k.twist.linear.x = 0;
    k.twist.linear.y = 0.1;
    k.twist.angular.z = 0;
    k.header.stamp += ros::Duration(2.0*M_PI*R*0.25*2.0/((0.1)+(Vmax/2.)));
    seg.push_back(k);
    // Come to a complete stop in a straight line after 0.5 sec
    k.pose.position.y+= 0.5*(0.1/0.5)*pow(0.5, 2.0);
    k.twist.linear.y = 0.0;
    k.header.stamp += ros::Duration(0.5);
    seg.push_back(k);

    return seg;
}

std::list<platform_motion_msgs::Knot> rightSpin()
{
    std::list<platform_motion_msgs::Knot> seg;
    platform_motion_msgs::Knot k;
    // set identity start point, stopped
    k.pose.orientation.w=1.0;
    seg.push_back(k);

    // rotate by 0.01 rad, accelerating to 0.01rad/sec
    // This should give the steering plenty of time to get around
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw( 0.01 ), k.pose.orientation);
    k.twist.angular.z = 0.01;
    k.header.stamp += ros::Duration(10.0);
    seg.push_back(k);

    // rotate to pi/4, accelerating to 0.7rad/sec over 1.0 sec
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw( M_PI_4 ), k.pose.orientation);
    k.twist.angular.z = 0.7;
    k.header.stamp += ros::Duration(5.0);
    seg.push_back(k);

    // rotate to pi/2, decelerating back to 0 angular rate
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw( M_PI_2 ), k.pose.orientation);
    k.twist.angular.z = 0.0;
    k.header.stamp += ros::Duration(1.0);
    seg.push_back(k);

    // straighten out the wheels by accelerating to 0.01m/s
    // over 2.0s
    k.twist.linear.y = 0.01;
    k.header.stamp += ros::Duration(10.0);
    seg.push_back(k);

    // and come to a stop
    k.twist.linear.y = 0.0;
    k.header.stamp += ros::Duration(0.3);
    seg.push_back(k);

    return seg;
}

void TestPath::testMultipleSegments()
{
    std::list<platform_motion_msgs::Knot> seg;

    platform_motion_msgs::Path path;
    path.header.stamp = ros::Time(0);
    path.header.frame_id = "base_link";
    int seq=0;
    for(int i=0;i<4;i++ )
    {
        // send line ending in left turn
        seg = leftHook();
        path.knots.clear();
        path.knots.insert(path.knots.end(), seg.begin(), seg.end());

        std::transform( path.knots.begin(), path.knots.end(), path.knots.begin(),
                [&seq](platform_motion_msgs::Knot k)
                {
                    k.header.seq = seq++;
                    return k;
                });
        ROS_INFO_STREAM( "Sending hook " << i << " " << path.knots.size() );
        path_pub_.publish(path);
        // wait for stitched path to come back
        if(!waitForStitched(std::chrono::seconds(2)))
        {
            ROS_ERROR( "Timeout waiting for stitched hook %d", i);
            break;
        }
        // copy knot at beginning of turn
        if( stitched_knots_.size() < 5 )
        {
            ROS_ERROR( "stitched path unexpectedly short: %ld", stitched_knots_.size() );
            break;
        }
        std::unique_lock<std::mutex> lck(knot_mutex_);
        platform_motion_msgs::Knot replace = *(stitched_knots_.end()-4);
        platform_motion_msgs::Knot previous = *(stitched_knots_.end()-5);
        lck.unlock();
        ROS_INFO_STREAM( "Replace at " << replace.header.stamp );
        ROS_INFO_STREAM( "Previous at " << previous.header.stamp );

        // compute a right spin in place
        seg = rightSpin();
        ROS_INFO( "Computed rightSpin %ld", seg.size() );
        // offset time so it inserts immediately before beginning of turn
        ros::Duration dt = replace.header.stamp - ros::Time(0.01);

        // offset space so it is in the same place as the replaced segment
        Eigen::Vector3d rx, px, dx;
        tf::pointMsgToEigen(replace.pose.position, rx);
        tf::pointMsgToEigen(previous.pose.position, px);
        Eigen::Quaterniond pq;
        tf::quaternionMsgToEigen( previous.pose.orientation, pq );
        dx = pq.inverse()*(rx-px);
        path.knots.resize(seg.size());
        std::transform( seg.begin(), seg.end(), path.knots.begin(),
                [dt, dx]( platform_motion_msgs::Knot k )
                {
                    k.header.stamp += dt;
                    Eigen::Vector3d kx;
                    tf::pointMsgToEigen( k.pose.position, kx );
                    tf::pointEigenToMsg( kx+dx, k.pose.position );
                    return k;
                });
        /*
         * This is the transfrom inside motion.cpp
        orientation = orientation*previous.rotation;
        position = previous.translation + previous.rotation*position;
        kvelocity = previous.rotation*kvelocity;
        komega = previous.rotation*komega;
        */

        ROS_INFO( "appended transformed points %ld", path.knots.size() );
        // send turn in place
        std::transform( path.knots.begin(), path.knots.end(), path.knots.begin(),
                [&seq](platform_motion_msgs::Knot k)
                {
                    k.header.seq = seq++;
                    return k;
                });
        ROS_INFO( "Sending spin %d %ld", i, path.knots.size() );
        path_pub_.publish(path);
        // wait for new stitched path. Need to wait here so
        // the next wait is really waiting for the immediately preceding
        // send
        if( !waitForStitched( std::chrono::seconds(1) ) )
        {
            ROS_ERROR( "Timeout waiting for stitched spin %d", i);
            break;
        }
    }
}

std::list<platform_motion_msgs::Knot> TestPath::computeStraightPath(double tconst)
{
    std::list<platform_motion_msgs::Knot> retval;

    platform_motion_msgs::Knot knot;
    knot.pose.orientation.w=1.0;

    ros::Duration step = ros::Duration(dt_);

    while(knot.twist.linear.x<maxv_)
    {
        retval.push_back(knot);
        knot.header.stamp += step;
        knot.header.seq += 1;
        double t = (knot.header.stamp).toSec();
        knot.twist.linear.x  = acceleration_*t;
        knot.pose.position.x = dt_*acceleration_*t*t;
    }
    knot = retval.back();
    knot.twist.linear.x = maxv_;
    for(double t=0;t<tconst;t+=dt_) {
        knot.header.stamp += step;
        knot.header.seq += 1;
        knot.pose.position.x += maxv_*dt_;
        retval.push_back(knot);
    }
    ros::Time tdecel=knot.header.stamp;
    double xdecel=knot.pose.position.x;
    while(knot.twist.linear.x>0)
    {
        knot.header.stamp += step;
        knot.header.seq += 1;
        double t = (knot.header.stamp - tdecel).toSec();
        knot.twist.linear.x  = maxv_ - acceleration_*t;
        knot.pose.position.x = xdecel + (maxv_*t - dt_*acceleration_*t*t);
        if(knot.twist.linear.x>0) retval.push_back(knot);
    }
    knot = retval.back();
    double decelrest=knot.twist.linear.x/acceleration_;
    knot.header.stamp    += ros::Duration(decelrest);
    knot.pose.position.x += dt_*acceleration_*decelrest*decelrest;
    knot.twist.linear.x   = 0;
    retval.push_back(knot);

    return retval;
}

std::list<platform_motion_msgs::Knot> TestPath::computeCirclePath(double R )
{
    std::list<platform_motion_msgs::Knot> retval;

    platform_motion_msgs::Knot knot;
    knot.pose.orientation.w=1.0;

    ros::Duration step = ros::Duration(dt_);

    double s=0, v=0, t=0, theta=0;
    retval.push_back(knot);
    while(v<maxv_)
    {
        knot.header.stamp += step;
        knot.header.seq += 1;
        t += dt_;
        v = acceleration_*t;
        s = dt_*acceleration_*t*t;
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
        t += dt_;
        s += v*dt_;
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
        t += dt_;
        v  = vdecel-acceleration_*t;
        s  = sdecel+vdecel*t-dt_*acceleration_*t*t;
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

std::list<platform_motion_msgs::Knot> TestPath::computeFigureEight(int numPoints, double amplitude, double omega)
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
    double startDistance = 0.5 * (pow(xDot0, 2.0) + pow(yDot0, 2.0)) / acceleration_;
    double startDuration = sqrt(pow(xDot0, 2.0) + pow(yDot0, 2.0)) / acceleration_;
    platform_motion_msgs::Knot start;
    start.pose.orientation.w=1.0;
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
        / acceleration_;
    double endDuration = sqrt(pow(retval.back().twist.linear.x, 2.0) + pow(retval.back().twist.linear.y, 2.0)) /
        acceleration_;

    end.header.stamp = retval.back().header.stamp + ros::Duration(endDuration);
    end.header.seq = seq++;
    end.twist.linear.x = 0.0;
    end.twist.linear.y = 0.0;
    end.twist.angular.z = 0.0;
    end.pose.position.x = endDistance * cos(theta) + retval.back().pose.position.x;
    end.pose.position.y = endDistance * sin(theta) + retval.back().pose.position.y;
    end.pose.orientation = retval.back().pose.orientation;

    retval.push_back(end);

    double xoff, yoff;
    xoff = retval.front().pose.position.x;
    yoff = retval.front().pose.position.y;
    for( auto k : retval )
    {
        k.pose.position.x -= xoff;
        k.pose.position.y -= yoff;
    }

    return retval;
}

std::list<platform_motion_msgs::Knot> TestPath::computeScaryPath(int which)
{
    std::list<platform_motion_msgs::Knot> retval;

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
            ROS_INFO( "Computing circle path." );
            retval=computeCirclePath(R);
            break;
        case platform_motion_msgs::ScaryTestMode::Straight:
            ROS_INFO( "Computing straight path." );
            retval=computeStraightPath(tconst);
            break;
        case platform_motion_msgs::ScaryTestMode::FigureEight:
            retval=computeFigureEight(numPoints, amplitude, omega);
            break;
        case platform_motion_msgs::ScaryTestMode::Spin:
            retval=rightSpin();
            break;
        case platform_motion_msgs::ScaryTestMode::Hook:
            retval=leftHook();
            break;
        case platform_motion_msgs::ScaryTestMode::Box:
            // testMultipleSegments does the sending, so there's no return value here
            // this will also test the behavior of sending an empty path, which should
            // work, but is currently untested!
            ROS_INFO( "Sending multiple segments." );
            testMultipleSegments();
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
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

