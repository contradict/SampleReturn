#include "odometry/least_squares_odometry_node.h"
#include "odometry/least_squares_odometry_lmmin.h"

namespace platform_motion
{

LeastSquaresOdometryNode::LeastSquaresOdometryNode(void):OdometryNode()
{
    ros::NodeHandle param_nh("~");

    param_nh.param("leastsquares_odometry", leastsquares_odometry, true);
    param_nh.param("leastsquares_velocity", leastsquares_velocity, false);

}

void LeastSquaresOdometryNode::computeOdometry(struct odometry_measurements &data, const ros::Time &stamp)
{
    bool moving = isMoving(data);
    if(moving)
    {
        double xythetavxvyomega[5] =
        {0.1*cos(odom_orientation)*data.interval, 0.1*sin(odom_orientation)*data.interval,
            odom_omega*data.interval, odom_velocity.norm(), odom_omega};
        lm_status_struct status;
        lm_control_struct control = lm_control_double;
        if(leastsquares_velocity)
        {
            lm_status_struct status;
            lm_control_struct control = lm_control_double;
            control.maxcall=500;
            control.printflags = 0;
            lmmin( 5, xythetavxvyomega, 10, &data, lmmin_evaluate_velocity, &control, &status, NULL);
        }
        else
        {
            control.maxcall=500;
            control.printflags = 0;
            lmmin( 3, xythetavxvyomega, 6, &data, lmmin_evaluate, &control, &status, NULL);
            xythetavxvyomega[4] = xythetavxvyomega[0]/data.interval;
            xythetavxvyomega[5] = xythetavxvyomega[1]/data.interval;
            xythetavxvyomega[6] = xythetavxvyomega[2]/data.interval;
        }
        if(status.info>4) {

            // minimization failed
            ROS_ERROR( "%s", (std::string("Minimization failed: ") +
                        lm_infmsg[status.info]).c_str() );
        }
        else
        {
            //ROS_DEBUG( "optimization complete %d: %s, %d evaluations", status.info, lm_infmsg[status.info], status.nfev );
            //ROS_DEBUG( "optimum(%e): %e %e %e %e %e", status.fnorm, xytheta[0], xytheta[1], xytheta[2], xytheta[3], xytheta[4]);
            Eigen::Matrix2d R;
            R << cos(odom_orientation), -sin(odom_orientation),
              sin(odom_orientation),  cos(odom_orientation);
            Eigen::Vector2d T=R*Eigen::Vector2d(xythetavxvyomega[0], xythetavxvyomega[1]);
            odom_position += T;
            odom_orientation += xythetavxvyomega[2];
            if(odom_orientation > 2*M_PI) odom_orientation -= 2*M_PI;
            if(odom_orientation < 0) odom_orientation += 2*M_PI;
            if(T.norm() > min_translation_norm)
                odom_velocity = xythetavxvyomega[3]*T.normalized();
            else
                odom_velocity = Eigen::Vector2d::Zero();
            odom_omega = xythetavxvyomega[4];

            resetReferencePose(data, stamp);
        }
    }
    else
    { // not moving
        if( fabs(data.port_vel) < 1e-3 &&
                fabs(data.starboard_vel) < 1e-3 &&
                fabs(data.stern_vel) < 1e-3)
        {
            odom_velocity = Eigen::Vector2d::Zero();
            odom_omega = 0;
        }
    }
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");

    platform_motion::LeastSquaresOdometryNode on;

    on.init();

    ros::spin();

    return 0;
}
