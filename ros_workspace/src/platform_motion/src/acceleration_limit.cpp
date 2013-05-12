#include "motion/acceleration_limit.h"
#include <iostream>
#include <math.h>

namespace platform_motion {

const double ACCEL_LIMIT_EPS=1e-3;

Eigen::Vector2d computePodVelocity(
        Eigen::Vector3d body_velocity,
        Eigen::Vector2d position
        )
{
    Eigen::Vector2d pod_velocity;
    pod_velocity(0) = body_velocity(0) - body_velocity(2)*position(1);
    pod_velocity(1) = body_velocity(1) + body_velocity(2)*position(0);
    return pod_velocity;
}

Eigen::Vector3d computeMatrixRow(
        Eigen::Vector3d body_velocity,
        Eigen::Vector2d position
        )
{
    Eigen::Vector2d V = computePodVelocity(body_velocity, position);
    //std::cout << "    " << "V      " << V << std::endl;
    double r = V.norm();
    if ( r < ACCEL_LIMIT_EPS )
    {
        return (Eigen::Vector3d() << NAN, NAN, NAN).finished();
    }
    double sign=1.0;
    if(V(0)<0) {
        V *= -1.0;
        sign = -1.0;
    }
    double atan2_ = V(1)/(r + V(0));
    //std::cout << "    " << "atan2_ " << atan2_ << std::endl;
    double coeff1 = 2./(1.+pow(atan2_,2.0));
    //std::cout << "    " << "coeff1 " << coeff1 << std::endl;
    double A, B, E;
    A = coeff1*(1./(r + V(0)) - pow(atan2_,2.0)/r);
    //std::cout << "    " << "A      " << A << std::endl;
    B = coeff1*-1.0*V(1)/pow(r + V(0), 2.0)*(1.+V(0)/r);
    //std::cout << "    " << "B      " << B << std::endl;
    E = A*position(0)-B*position(1);
    //std::cout << "    " << "E      " << C << std::endl;
    return (Eigen::Vector3d() << B, A, E).finished()*sign;
}

Eigen::Vector3d VelocityBounds(
        Eigen::Vector3d body_velocity,
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double max_steering_omega
        )
{
    Eigen::Matrix3d m;
    //std::cout << "stern" << std::endl;
    m.row(0) = computeMatrixRow(body_velocity, stern_position);
    //std::cout << "  " << "row " << m.row(0) << std::endl;
    //std::cout << "starboard" << std::endl;
    m.row(1) = computeMatrixRow(body_velocity, starboard_position);
    //std::cout << "  " << "row " << m.row(1) << std::endl;
    //std::cout << "port" << std::endl;
    m.row(2) = computeMatrixRow(body_velocity, port_position);
    //std::cout << "  " << "row " << m.row(2) << std::endl;
    std::cout <<  "m " << std::endl << m << std::endl;

    Eigen::Vector3d bounds;
    return m.fullPivLu().solve(Eigen::Vector3d::Constant(max_steering_omega));
}

Eigen::Vector3d SelectClosestVelocity(
        Eigen::Vector3d body_velocity,
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double max_steering_omega,
        Eigen::Vector3d deltav,
        Eigen::Vector3d desired_velocity)
{
    Eigen::Vector3d bounds = VelocityBounds(
            body_velocity,
            stern_position,
            starboard_position,
            port_position,
            max_steering_omega);
    bounds = bounds.cwiseAbs();
    bounds = bounds.cwiseMin(deltav);
    for(int i=0;i<3;i++)
    {
        if(isnan(bounds(i)))
            bounds(i) = deltav(i);
    }

    Eigen::Vector3d delta=desired_velocity-body_velocity;
    Eigen::Vector3d abs_delta=delta.cwiseAbs();
    Eigen::Vector3d bounded_abs_delta=abs_delta.cwiseMin(bounds);
    Eigen::Vector3d achieved;
    for(int i=0;i<3;i++)
    {
        achieved(i)=copysign(bounded_abs_delta(i), delta(i));
    }
    return achieved;
}

/*
 * number with same sign as a but absolute value equal to minimum of
 * abs(a) and b
 */
double limit(double a, double b)
{
    return copysign(std::min(fabs(a), b), a);
}

Eigen::Vector3d podLimit(
        Eigen::Vector2d pod_position,
        double dtheta,
        double current_angle,
        Eigen::Vector3d desired_body_velocity
        )
{
    std::cout << "pod_position: " << pod_position.transpose() << std::endl;
    Eigen::Vector2d desired_pod_velocity = computePodVelocity(desired_body_velocity, pod_position);
    std::cout << "desired_pod_velocity: " << desired_pod_velocity.transpose() << std::endl;
    double desired_pod_speed = desired_pod_velocity.norm();
    double desired_angle = atan2(desired_pod_velocity(1), desired_pod_velocity(0));
    if(desired_angle<-M_PI_2)
        desired_angle += M_PI;
    if(desired_angle>M_PI_2)
        desired_angle -= M_PI;
    if(current_angle<0)
        current_angle += M_PI;
    if(current_angle>M_PI_2)
        current_angle -= M_PI;
    Eigen::Vector2d desired_pod_direction;
    desired_pod_direction << cos(desired_angle), sin(desired_angle);
    double delta_angle = desired_angle-current_angle;
    double achievable_angle = current_angle + limit(delta_angle, dtheta);
    std::cout << "desired_angle: " << desired_angle << \
        " current_angle: " << current_angle << \
        " achievable_angle: " << achievable_angle << \
        std::endl;
    Eigen::Vector2d achievable_pod_direction;
    achievable_pod_direction << cos(achievable_angle), sin(achievable_angle);
    Eigen::Vector2d achievable_pod_velocity = achievable_pod_direction*desired_pod_speed;
    std::cout << "achievable_pod_velocity: " << achievable_pod_velocity.transpose() << std::endl;
    /*
       for(int i=0;i<2;i++)
       {
       achievable_pod_velocity(i) = limit(desired_pod_velocity(i), fabs(achievable_pod_velocity(i)));
       }
       std::cout << "achievable_pod_velocity(limited): " << achievable_pod_velocity.transpose() << std::endl;
       */
    Eigen::Vector2d pod_deltav=achievable_pod_velocity - desired_pod_velocity;
    double pod_deltaspeed=pod_deltav.norm();
    std::cout << "pod_deltav: " << pod_deltav.transpose() << std::endl;

    Eigen::Vector3d body_deltav;
    double desired_body_speed=desired_body_velocity.head(2).norm();
    if(desired_body_speed<1e-4 || pod_deltaspeed<1e-4)
    {
        body_deltav(0) = 0.0;
        body_deltav(1) = 0.0;
    }
    else
    {
        Eigen::Vector2d desired_body_direction=desired_body_velocity.head(2)/desired_body_speed;
        body_deltav.head(2) = -pod_deltav.dot(desired_body_direction)*pod_deltav/pod_deltaspeed;
    }

    Eigen::Vector2d omega_deltav = body_deltav.head(2) - pod_deltav;
    std::cout << "omega_deltav: " << omega_deltav.transpose() << std::endl;
    if(fabs(pod_position(0))>1e-4)
        body_deltav(2) = -omega_deltav(1)/pod_position(0);
    else
        body_deltav(2) = omega_deltav(0)/pod_position(1);
    std::cout << "body_deltav: " << body_deltav.transpose() << std::endl;
    return body_deltav;
}

Eigen::Vector3d SimpleLimit(
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double dtheta,
        double stern_angle,
        double starboard_angle,
        double port_angle,
        Eigen::Vector3d desired_body_velocity
        )
{
    Eigen::Vector3d stern_deltav, starboard_deltav, port_deltav;
    stern_deltav = podLimit(stern_position, dtheta, stern_angle, desired_body_velocity);
    std::cerr << "stern deltav: " << stern_deltav.transpose() << std::endl;
    starboard_deltav = podLimit(starboard_position, dtheta, starboard_angle, desired_body_velocity);
    std::cerr << "starboard deltav: " << starboard_deltav.transpose() << std::endl;
    port_deltav = podLimit(port_position, dtheta, port_angle, desired_body_velocity);
    std::cerr << "port deltav: " << port_deltav.transpose() << std::endl;

    Eigen::Vector3d deltav;
    for(int i=0;i<3;i++)
    {
        if(fabs(starboard_deltav(i))>fabs(port_deltav(i)))
            deltav(i) = starboard_deltav(i);
        else
            deltav(i) = port_deltav(i);
        if(fabs(stern_deltav(i))>fabs(deltav(i)))
            deltav(i) = stern_deltav(i);
    }
    std::cerr << "deltav: " << deltav.transpose() << std::endl;
    std::cerr << "desired: " << desired_body_velocity.transpose() << std::endl;
    Eigen::Vector3d achievable = desired_body_velocity+deltav;
    std::cerr << "achievable: " << achievable.transpose() << std::endl;

    return achievable;
}

void computePod(
        Eigen::Vector2d pod_position,
        Eigen::Vector3d desired_body_velocity,
        double *pod_angle,
        double *pod_speed)
{
    Eigen::Vector2d desired_pod_velocity = computePodVelocity(desired_body_velocity, pod_position);
    *pod_angle = atan2(desired_pod_velocity(1), desired_pod_velocity(0));
    *pod_speed = desired_pod_velocity.norm();
}

double flipVelocity(double angle)
{
    if(angle>M_PI_2)
        angle -= M_PI;
    if(angle<-M_PI_2)
        angle += M_PI;
    return angle;
}

Eigen::Vector3d NumericalLimit(
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double dtheta,
        double min_step,
        double tolerance,
        double stern_angle,
        double starboard_angle,
        double port_angle,
        Eigen::Vector3d present_body_velocity,
        Eigen::Vector3d desired_body_velocity
        )
{
    if((present_body_velocity - desired_body_velocity).norm()<0.01)
        return desired_body_velocity;
    double desired_stern_angle, desired_starboard_angle, desired_port_angle;
    double desired_stern_speed, desired_starboard_speed, desired_port_speed;
    computePod(stern_position, desired_body_velocity, &desired_stern_angle, &desired_stern_speed);
    computePod(starboard_position, desired_body_velocity, &desired_starboard_angle, &desired_starboard_speed);
    computePod(port_position, desired_body_velocity, &desired_port_angle, &desired_port_speed);

    stern_angle = flipVelocity(stern_angle);
    starboard_angle = flipVelocity(starboard_angle);
    port_angle = flipVelocity(port_angle);
    desired_stern_angle = flipVelocity(desired_stern_angle);
    desired_starboard_angle = flipVelocity(desired_starboard_angle);
    desired_port_angle = flipVelocity(desired_port_angle);

    std::cerr << "stern: " << stern_angle << " starboard: " << starboard_angle << " port: " << port_angle << std::endl;
    std::cerr << "stern: " << desired_stern_angle << " starboard: " << desired_starboard_angle << " port: " << desired_port_angle << std::endl;

    double stern_dtheta, starboard_dtheta, port_dtheta;
    stern_dtheta = desired_stern_angle - stern_angle;
    starboard_dtheta = desired_starboard_angle - starboard_angle;
    port_dtheta = desired_port_angle - port_angle;

    Eigen::Vector2d limiting_position;
    double limiting_angle;
    double limiting_dtheta;
    if(fabs(stern_dtheta)>fabs(starboard_dtheta))
        if(fabs(stern_dtheta)>fabs(port_dtheta))
        {
            limiting_position = stern_position;
            limiting_angle = stern_angle;
            limiting_dtheta = stern_dtheta;
            std::cerr << "stern limiting: " << limiting_dtheta << std::endl;
        }
        else
        {
            limiting_position = port_position;
            limiting_angle = port_angle;
            limiting_dtheta = port_dtheta;
            std::cerr << "port limiting: " << limiting_dtheta << std::endl;
        }
    else
        if(fabs(starboard_dtheta)>fabs(port_dtheta))
        {
            limiting_position = starboard_position;
            limiting_angle = starboard_angle;
            limiting_dtheta = starboard_dtheta;
            std::cerr << "starboard limiting: " << limiting_dtheta << std::endl;
        }
        else
        {
            limiting_position = port_position;
            limiting_angle = port_angle;
            limiting_dtheta = port_dtheta;
            std::cerr << "port limiting: " << limiting_dtheta << std::endl;
        }
    Eigen::Vector3d achievable_body_velocity=desired_body_velocity;
    double err=fabs(limiting_dtheta) - dtheta;
    double a=1.0;
    std::cerr << "a: " << a << " err: " << err << std::endl;
    int its=0;
    while( err>tolerance && a>min_step)
    {
        double limiting_theta, limiting_speed;
        if(err>0)
            a/=2;
        else
            a*=M_SQRT2;
        //std::cerr << "a: " << a << std::endl;
        //std::cerr << "present: " << present_body_velocity.transpose() << std::endl;
        //std::cerr << "desired: " << desired_body_velocity.transpose() << std::endl;
        achievable_body_velocity = present_body_velocity + a*(desired_body_velocity - present_body_velocity);
        //std::cerr << "achievable: " << achievable_body_velocity.transpose() << std::endl;
        computePod(limiting_position, achievable_body_velocity, &limiting_theta, &limiting_speed);
        limiting_theta = flipVelocity(limiting_theta);
        limiting_dtheta = limiting_theta - limiting_angle;
        //std::cerr << "limiting_theta: " << limiting_theta << " limiting_angle: " << limiting_angle << " limiting_dtheta: " <<limiting_dtheta << std::endl;
        err=fabs(limiting_dtheta) - dtheta;
        //std::cerr << "err: " << err << std::endl;
        its++;
    }
    std::cerr << "present: " << present_body_velocity.transpose() << std::endl;
    std::cerr << "desired: " << desired_body_velocity.transpose() << std::endl;
    std::cerr << "its: " << its << " a: " << a << std::endl;
    std::cerr << "achievable: " << achievable_body_velocity.transpose() << std::endl;
    return achievable_body_velocity;
}

}
