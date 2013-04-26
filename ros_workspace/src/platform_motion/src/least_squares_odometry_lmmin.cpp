#include "odometry/least_squares_odometry_lmmin.h"
#include "odometry/odometry_node.h"

namespace platform_motion {

Eigen::Vector2d computePositionError(
        Eigen::Matrix2d R, Eigen::Vector2d T,
        Eigen::Vector2d dir, double delta, Eigen::Vector2d pos,
        Eigen::Vector2d body_pt)
{
   return R*(pos - body_pt) + body_pt + T - (pos + dir*delta);
}

double computeVelocityError(
        Eigen::Matrix2d Rdot, Eigen::Vector2d V,
        Eigen::Vector2d dir, double vel, Eigen::Vector2d pos,
        Eigen::Vector2d body_pt)
{
   return (Rdot*(pos - body_pt) + V).dot(dir) - vel;
}

void lmmin_evaluate_velocity(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info)
{
    const struct odometry_measurements *data = reinterpret_cast<const struct odometry_measurements *>(vdata);

    double x,y,theta,v,omega;

    x=xytheta[0];
    y=xytheta[1];
    theta=xytheta[2];
    v=xytheta[3];
    omega=xytheta[4];

    /*
    ROS_DEBUG("x: %e y: %e theta: %e v: %e omega: %e", x, y, theta, v, omega);
    ROS_DEBUG_STREAM("body_pt:" << data->body_pt.transpose());
    ROS_DEBUG_STREAM("port_pos:" << data->port_pos.transpose() <<
                     " port_dir:" << data->port_dir);
    ROS_DEBUG_STREAM("starboard_pos:" << data->starboard_pos.transpose() <<
                     " starboard_dir:" << data->starboard_dir);
    ROS_DEBUG_STREAM("stern_pos:" << data->stern_pos.transpose() <<
                     " stern_dir:" << data->stern_dir);
    */

    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta),  cos(theta);

    Eigen::Vector2d T(x, y);
    Eigen::Vector2d V;
    if( T.norm() > data->min_translation_norm)
        V=v*T.normalized();
    else
        V=Eigen::Vector2d::Zero();
    //ROS_DEBUG_STREAM("V: " << V.transpose() );

    Eigen::Vector2d port_error = computePositionError(R, T, data->port_dir, data->port_delta, data->port_pos, data->body_pt);
    //ROS_DEBUG("port_delta(%f) port_error(%f %f)",
    //        data->port_delta,
    //        port_error[0], port_error[1]);
    Eigen::Vector2d starboard_error = computePositionError(R, T, data->starboard_dir, data->starboard_delta, data->starboard_pos, data->body_pt);
    //ROS_DEBUG("starboard_delta(%f) starboard_error(%f %f)",
    //        data->starboard_delta,
    //        starboard_error[0], starboard_error[1]);
    Eigen::Vector2d stern_error = computePositionError(R, T, data->stern_dir, data->stern_delta, data->stern_pos, data->body_pt);
    //ROS_DEBUG("stern_delta(%f) stern_error(%f %f)",
    //        data->stern_delta,
    //        stern_error[0], stern_error[1]);

    Eigen::Matrix2d Rdot;
    Rdot << -sin(theta)*omega, -cos(theta)*omega,
             cos(theta)*omega, -sin(theta)*omega;
    double port_velocity_error = computeVelocityError(Rdot, V, data->port_dir, data->port_vel, data->port_pos, data->body_pt);
    //ROS_DEBUG("port_vel(%f) port_velocity_error(%f)", data->port_vel, port_velocity_error);
    double starboard_velocity_error = computeVelocityError(Rdot, V, data->starboard_dir, data->starboard_vel, data->starboard_pos, data->body_pt);
    //ROS_DEBUG("starboard_vel(%f) starboard_velocity_error(%f)", data->starboard_vel, starboard_velocity_error);
    double stern_velocity_error = computeVelocityError(Rdot, V, data->stern_dir, data->stern_vel, data->stern_pos, data->body_pt);
    //ROS_DEBUG("stern_vel(%f) stern_velocity_error(%f)", data->stern_vel, stern_velocity_error);

    fvec[0] = port_error(0);
    fvec[1] = port_error(1);
    fvec[2] = starboard_error(0);
    fvec[3] = starboard_error(1);
    fvec[4] = stern_error(0);
    fvec[5] = stern_error(1);
    fvec[6] = port_velocity_error;
    fvec[7] = starboard_velocity_error;
    fvec[8] = stern_velocity_error;
    fvec[9] = v-T.norm()/data->interval;

    //ROS_DEBUG( "fvec: [%f %f %f %f %f %f %f %f %f %f]",
    //        fvec[0], fvec[1], fvec[2],
    //        fvec[3], fvec[4], fvec[5],
    //        fvec[6], fvec[7], fvec[8],
    //        fvec[9]);

    *info = 1;
}

void lmmin_evaluate(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info)
{
    const struct odometry_measurements *data = reinterpret_cast<const struct odometry_measurements *>(vdata);

    double x,y,theta;

    x=xytheta[0];
    y=xytheta[1];
    theta=xytheta[2];

    /*
    ROS_DEBUG("x: %e y: %e theta: %e v: %e omega: %e", x, y, theta, v, omega);
    ROS_DEBUG_STREAM("body_pt:" << data->body_pt.transpose());
    ROS_DEBUG_STREAM("port_pos:" << data->port_pos.transpose() <<
                     " port_dir:" << data->port_dir);
    ROS_DEBUG_STREAM("starboard_pos:" << data->starboard_pos.transpose() <<
                     " starboard_dir:" << data->starboard_dir);
    ROS_DEBUG_STREAM("stern_pos:" << data->stern_pos.transpose() <<
                     " stern_dir:" << data->stern_dir);
    */

    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta),  cos(theta);

    Eigen::Vector2d T(x, y);

    Eigen::Vector2d port_error = computePositionError(R, T, data->port_dir, data->port_delta, data->port_pos, data->body_pt);
    //ROS_DEBUG("port_delta(%f) port_error(%f %f)",
    //        data->port_delta,
    //        port_error[0], port_error[1]);
    Eigen::Vector2d starboard_error = computePositionError(R, T, data->starboard_dir, data->starboard_delta, data->starboard_pos, data->body_pt);
    //ROS_DEBUG("starboard_delta(%f) starboard_error(%f %f)",
    //        data->starboard_delta,
    //        starboard_error[0], starboard_error[1]);
    Eigen::Vector2d stern_error = computePositionError(R, T, data->stern_dir, data->stern_delta, data->stern_pos, data->body_pt);
    //ROS_DEBUG("stern_delta(%f) stern_error(%f %f)",
    //        data->stern_delta,
    //        stern_error[0], stern_error[1]);

    fvec[0] = port_error(0);
    fvec[1] = port_error(1);
    fvec[2] = starboard_error(0);
    fvec[3] = starboard_error(1);
    fvec[4] = stern_error(0);
    fvec[5] = stern_error(1);

    //ROS_DEBUG( "fvec: [%f %f %f %f %f %f %f %f %f %f]",
    //        fvec[0], fvec[1], fvec[2],
    //        fvec[3], fvec[4], fvec[5],
    //        fvec[6], fvec[7], fvec[8],
    //        fvec[9]);

    *info = 1;
}


}
