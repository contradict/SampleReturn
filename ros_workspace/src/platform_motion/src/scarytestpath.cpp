#include <limits>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <platform_motion_msgs/Path.h>
#include <platform_motion_msgs/ScaryTestMode.h>

namespace platform_motion {

const double LARGE_RADIUS = 2e3;
const double SMALL_RADIUS = 0.01;

class Circle
{
    double kappa_;
    double R_;
    double v_;
    double omega_;

public:

    double time_;
    double yaw_;
    double theta_;
    Eigen::Vector3d pos_;

    void curvature(double k)
    {
        kappa_ = k;
        if( k>1./SMALL_RADIUS )
        {
            R_ = 1./k;
        }
        else
        {
            R_ = std::numeric_limits<double>::infinity();
        }
    }

    void radius(double R)
    {
        R_ = R;
        if( R>SMALL_RADIUS )
        {
            kappa_ = 1./R_;
        }
        else
        {
            kappa_ = std::numeric_limits<double>::infinity();
        }
    }

    double curvature(void) const
    {
        if( isinf( kappa_ ) )
        {
            if( R_ == 0)
            {
                return kappa_;
            }
            else
            {
                return 1./R_;
            }
        }
        else
        {
            return kappa_;
        }
    }

    double radius(void) const
    {
        if( isinf(R_) )
        {
            return 1./kappa_;
        }
        else
        {
            return R_;
        }
    }

    void omega(double w)
    {
        omega_ = w;
        if( isinf(kappa_) )
        {
            v_ = omega_*R_;
        }
        else
        {
            v_ = omega_/kappa_;
        }
    }

    double omega(void) const
    {
        return omega_;
    };

    void speed(double v)
    {
        v_ = v;
        if( isinf(kappa_) && R_ != 0 )
        {
            omega_ = v_/R_;
        }
        else
        {
            omega_ = kappa_*v_;
        }
    };

    double speed(void) const
    {
        return v_;
    }

    Eigen::Vector3d velocity(void)
    {
        Eigen::Vector3d vel;
        vel << speed()*cos(theta_+yaw_-M_PI_2), speed()*sin(theta_+yaw_-M_PI_2), 0;
        return vel;
    };

    platform_motion_msgs::Knot knot(void)
    {
        platform_motion_msgs::Knot k;
        k.header.stamp = ros::Time( time_ );
        tf::pointEigenToMsg( pos_, k.pose.position );
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,yaw_), k.pose.orientation);
        tf::vectorEigenToMsg( velocity(), k.twist.linear );
        k.twist.angular.x = 0;
        k.twist.angular.y = 0;
        k.twist.angular.z = omega();
        return k;
    };

    Circle adjust(Circle target, double dt, double maxphidot, double acceleration, double alpha, double wheelbase)
    {
        Circle adjusted;
        double thetaerror = theta_ - target.theta_;
        double kappa, targetkappa;
        if( isinf(curvature()) )
        {
            kappa = 1./SMALL_RADIUS;
        }
        else
        {
            kappa = curvature();
        }
        if( isinf( target.curvature() ) )
        {
            targetkappa = 1./SMALL_RADIUS;
        }
        else
        {
            targetkappa = target.curvature();
        }
        double kappaerror = kappa - targetkappa;
        double phierror = thetaerror + kappaerror*wheelbase*(1.0+pow(kappa*wheelbase, 2.0));
        double newkappa;
        if(maxphidot*dt<fabs(phierror))
        {
            adjusted.theta_ = theta_ - copysign(maxphidot*dt*thetaerror/phierror, thetaerror);
            newkappa = kappa - kappaerror/phierror;
        }
        else
        {
            adjusted.theta_ = target.theta_;
            newkappa = target.curvature();
        }
        if( isinf(target.curvature()) && newkappa >= 1./SMALL_RADIUS)
        {
            adjusted.radius( 0.0 );
        }
        else
        {
            adjusted.curvature( newkappa );
        }
        if( target.R_ == 0 )
        {
            double werror = omega() - target.omega();
            double deltaw = std::min( dt*alpha, fabs(werror) );
            adjusted.omega( omega() + copysign( deltaw, werror ) );
        }
        else
        {
            double verror=speed() - target.speed();
            double deltav=std::min( dt*acceleration, fabs(verror) );
            adjusted.speed( speed() + copysign( deltav, verror ) );
        }
        Eigen::Vector3d dir;
        dir << cos(theta_+yaw_-M_PI_2), sin(theta_+yaw_-M_PI_2), 0;
        adjusted.pos_ = pos_ + dt*(speed()+adjusted.speed())/2.*dir;
        adjusted.yaw_ = yaw_ + dt*(omega() + adjusted.omega())/2.;
        adjusted.time_ = time_ + dt;
        return adjusted;
    };

    double distance(const Circle other)
    {
        return (pos_ - other.pos_).norm();
    };

};


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

        ros::Subscriber scary_test_mode_sub_;
        ros::Publisher path_pub_;

        tf::TransformListener listener;

        // scary pvt test mode enable. only do this on blocks!!!
        void scaryTestModeCallback(const platform_motion_msgs::ScaryTestMode::ConstPtr msg);
        std::list<platform_motion_msgs::Knot> computeCirclePath(double R);
        std::list<platform_motion_msgs::Knot> computeStraightPath(double tconst);
        std::list<platform_motion_msgs::Knot> computeFigureEight(int numPoints, double amplitude, double omega);
        std::list<platform_motion_msgs::Knot> computeScaryPath(int which);

        std::list<Circle> computeLeftHook(Circle initial, double distance, double radius );
        std::list<Circle> computeRightPivot(Circle initial, double dtheta);
        void testMultipleSegments();
};

TestPath::TestPath() :
    wheelbase_(1.125),
    acceleration_(0.1),
    maxv_(1.5),
    alpha_(0.13),
    maxomega_(2.0),
    maxphidot_(M_PI/2),
    dt_(0.5)
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
    path.header.stamp = ros::Time(0);
    path.header.frame_id = "map";

    std::list<platform_motion_msgs::Knot> knots = computeScaryPath(msg->which);
    path.knots.insert(path.knots.begin(), knots.begin(), knots.end() );

    path_pub_.publish(path);
}

std::list<Circle> TestPath::computeLeftHook(Circle initial, double distance, double radius)
{
    std::list<Circle> ret;

    ret.push_back( initial );

    Circle target;
    target.curvature( 0 );
    target.theta_ = M_PI_2;
    target.speed( maxv_ );

    Circle current = initial;

    ros::Time now = ros::Time(0);

    while( initial.distance(current)<distance )
    {
        current = current.adjust( target, dt_, maxphidot_, acceleration_, alpha_, wheelbase_ );
        ret.push_back( current );
    }

    target.radius( radius );

    while( (current.yaw_-initial.yaw_)<M_PI_2 )
    {
        target.speed( std::min( maxv_, (M_PI_2 - (current.yaw_ - initial.yaw_))/dt_ ) );
        current = current.adjust( target, dt_, maxphidot_, acceleration_, alpha_, wheelbase_ );
        ret.push_back( current );
    }

    return ret;
}

std::list<Circle> TestPath::computeRightPivot(Circle initial, double dtheta)
{
    std::list<Circle> ret;

    ret.push_back(initial);

    Circle current=initial;
    Circle target=initial;

    target.radius( 0.0 );
    target.theta_ = M_PI_2;
    target.yaw_ += dtheta;
    target.omega( maxomega_ );

    while( (current.yaw_ - initial.yaw_)<dtheta )
    {
        double yerror = (dtheta-(current.yaw_ - initial.yaw_)<dtheta );
        target.omega( copysign(std::min( maxomega_, yerror/dt_ ), dtheta) );
        current.adjust( target, dt_, maxphidot_, acceleration_, alpha_, wheelbase_ );
        ret.push_back( current );
    }

    return ret;
}

template<typename T, typename P>
T find_last_if( const std::list<T> &seg, P pred )
{
    return *std::find_if(
            typename std::list<T>::const_reverse_iterator( seg.end() ),
            typename std::list<T>::const_reverse_iterator( seg.begin() ),
            pred
            );
}

void TestPath::testMultipleSegments()
{
    std::vector<std::list<Circle> > segments;
    Circle start;
    start.curvature( 0 );
    start.speed( 0 );
    start.time_ = 0;
    start.yaw_ = 0;
    start.theta_ = M_PI_2;
    start.pos_ = Eigen::Vector3d::Zero();

    Circle lastStraight;
    for(int i=0;i<4;i++)
    {
        segments.push_back(computeLeftHook( start, 3.0, 1.5 ));

        lastStraight = find_last_if(
                segments.back(),
                [](Circle c){ return c.curvature() == 0; }
                );
        segments.push_back( computeRightPivot( lastStraight, -M_PI_2 ) );

        start = segments.back().back();
    }

    platform_motion_msgs::Path path;
    path.header.stamp = ros::Time(0);
    path.header.frame_id = "map";
    for( auto seg : segments )
    {
        path.header.stamp = ros::Time( seg.front().time_ );
        path.knots.clear();
        std::transform( seg.begin(), seg.end(), path.knots.begin(),
                [](Circle c)->platform_motion_msgs::Knot { return c.knot(); }
                );
        path_pub_.publish(path);

        ros::Duration( 1.0 ).sleep();
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
            retval=computeCirclePath(R);
            break;
        case platform_motion_msgs::ScaryTestMode::Straight:
            retval=computeStraightPath(tconst);
            break;
        case platform_motion_msgs::ScaryTestMode::FigureEight:
            retval=computeFigureEight(numPoints, amplitude, omega);
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

