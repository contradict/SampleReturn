#include "motion/acceleration_limit.h"
#include <iostream>
#include <math.h>

namespace platform_motion {

const double ACCEL_LIMIT_EPS=1e-3;

void flipVelocity(double *angle, double *velocity)
{
    if(*angle>M_PI_2)
    {
        *angle -= M_PI;
        *velocity *= -1;
    }
    if(*angle<=-M_PI_2)
    {
        *angle += M_PI;
        *velocity *= -1;
    }
}

Eigen::Vector2d computePodVelocity(
        const Eigen::Vector3d &body_velocity,
        const Eigen::Vector2d &position
        )
{
    Eigen::Vector2d pod_velocity;
    pod_velocity(0) = body_velocity(0) - body_velocity(2)*position(1);
    pod_velocity(1) = body_velocity(1) + body_velocity(2)*position(0);
    return pod_velocity;
}

void computePod(
        const Eigen::Vector2d &pod_position,
        const Eigen::Vector3d &desired_body_velocity,
        double *pod_angle,
        double *pod_speed)
{
    Eigen::Vector2d desired_pod_velocity = computePodVelocity(desired_body_velocity, pod_position);
    *pod_angle = atan2(desired_pod_velocity(1), desired_pod_velocity(0));
    *pod_speed = desired_pod_velocity.norm();
    flipVelocity(pod_angle, pod_speed);
}


void computePod(
        const Eigen::Vector2d &pod_position,
        const Eigen::Vector3d &center,
        const double omega,
        const double velocity,
        double *angle,
        double *wheel_velocity)
{
    Eigen::Vector3d zhat;
    zhat << 0,0,1;
    Eigen::Vector3d body_velocity;
    if(center(2) == 0.0)
    {
        body_velocity = zhat.cross(center)*velocity;
        body_velocity(2) = 0.0;
    }
    else
    {
        body_velocity = -center.cross(zhat*omega);
        body_velocity(2) = omega;
    }
    computePod(pod_position, body_velocity, angle, wheel_velocity);
}


Eigen::Vector3d bodyVelocityInterpolate(
        const double a,
        const Eigen::Vector3d &v1,
        const Eigen::Vector3d &v2)
{
    return a*v1 + (1.0-a)*v2;
}

#define MIN_OMEGA 0.01
void bodyVelocityToGeometric(
        const Eigen::Vector3d &body_velocity,
        Eigen::Vector3d &center,
        double *omega
        )
{
    *omega = body_velocity(2);
    double speed = body_velocity.head(2).norm();
    double radius = speed/fabs(*omega);
    Eigen::Vector3d zhat;
    zhat << 0,0,1;
    Eigen::Vector3d velocity3;
    velocity3.head(2) = body_velocity.head(2);
    velocity3(2) = 0.0;
    Eigen::Vector3d direction = zhat.cross(velocity3).normalized();
    if(*omega>MIN_OMEGA)
    {
        center = radius*direction;
        center(2) = 1.0;
    }
    else
    {
        center = direction;
        center(2) = 0.0;
        *omega = speed;
    }
}

#define PARALLEL_THRESHOLD 0.02
#define RADIUS_THRESHOLD 0.10
double podParamtersToGeometric(
        const Eigen::Vector2d &stern_position,
        const Eigen::Vector2d &starboard_position,
        const Eigen::Vector2d &port_position,
        double stern_angle, double stern_velocity,
        double starboard_angle, double starboard_velocity,
        double port_angle, double port_velocity,
        Eigen::Vector3d center,
        double *omega
        )
{
    double error=0;

    center << 0,0,0;
    if( fabs(stern_angle-starboard_angle) < PARALLEL_THRESHOLD &&
        fabs(stern_angle-port_angle) < PARALLEL_THRESHOLD &&
        fabs(starboard_angle-port_angle) < PARALLEL_THRESHOLD )
    {
        double direction = (stern_angle + starboard_angle + port_angle)/3.0;
        error = (fabs(stern_angle - direction) + fabs(starboard_angle-direction) + fabs(port_angle-direction))/3.0;
        double velocity = (stern_velocity + starboard_velocity + port_velocity)/3.0;
        center << cos(direction), sin(direction), 0;
        *omega = velocity;
    }
    else
    {
        Eigen::Vector3d stern_direction, starboard_direction, port_direction;
        Eigen::Vector3d stern_position_, starboard_position_, port_position_;

        stern_direction << cos(stern_angle), sin(stern_angle), 0;
        if(stern_velocity<0.0)
            stern_direction *= -1.0;
        starboard_direction << cos(starboard_angle), sin(starboard_angle), 0;
        if(starboard_velocity<0.0)
            starboard_direction *= -1.0;
        port_direction << cos(port_angle), sin(port_angle), 0;
        if(port_velocity<0.0)
            port_direction *= -1.0;

        stern_position_.head(2) = stern_position; stern_position_(2)=1.0;
        starboard_position_.head(2) = starboard_position; starboard_position_(2)=1.0;
        port_position_.head(2) = port_position; port_position_(2)=1.0;

        // line through pod position in direction pod is traveling.
        stern_direction(2) = -stern_direction.dot(stern_position_);
        starboard_direction(2) = -starboard_direction.dot(starboard_position_);
        port_direction(2) = -port_direction.dot(port_position_);

        Eigen::Vector3d i1, i2, i3;
        i1 = stern_direction.cross(starboard_direction);
        i2 = stern_direction.cross(port_direction);
        i3 = starboard_direction.cross(port_direction);

        int cnt=0;
        if( i1.norm()>PARALLEL_THRESHOLD )
        {
            i1 /= i1(2);
            center += i1;
            error += (i1-center).norm();
            cnt++;
        }
        if( i2.norm()>PARALLEL_THRESHOLD )
        {
            i2 /= i2(2);
            center += i2;
            error += (i2-center).norm();
            cnt++;
        }
        if( i3.norm()>PARALLEL_THRESHOLD )
        {
            i3 /= i3(2);
            center += i3;
            error += (i3-center).norm();
            cnt++;
        }
        center /= cnt;
        error /= cnt;

        cnt = 0;
        double stern_radius, starboard_radius, port_radius;
        stern_radius = (stern_position_ - center).norm();
        starboard_radius = (starboard_position_ - center).norm();
        port_radius = (port_position_ - center).norm();
        if( port_radius > RADIUS_THRESHOLD )
        {
            *omega += port_velocity/port_radius;
            cnt++;
        }
        if( starboard_radius > RADIUS_THRESHOLD )
        {
            *omega += starboard_velocity/starboard_radius;
            cnt++;
        }
        if( stern_radius > RADIUS_THRESHOLD )
        {
            *omega += stern_velocity/stern_radius;
            cnt++;
        }
        *omega /= cnt;
    }

    return error;

}

Eigen::Vector3d geometricToBody(
        const Eigen::Vector3d &center,
        const double omega
        )
{
    Eigen::Vector3d body_velocity;
    Eigen::Vector3d zhat;
    zhat << 0,0,1;
    body_velocity = center.cross(zhat);
    body_velocity(2) = 0;
    body_velocity = body_velocity.normalized();
    if(center(2) == 0.0)
        body_velocity *= omega;
    else
    {
        double radius=center.head(2).norm();
        body_velocity *= omega*radius;
        body_velocity(2) = omega;
    }
    return body_velocity;
}

#define MIN_SPEED 0.05
Eigen::Vector3d geometricInterpolate(
        double a,
        const Eigen::Vector3d present_body_velocity,
        const Eigen::Vector3d desired_body_velocity
        )
{
    double present_omega, desired_omega;
    Eigen::Vector3d present_center, desired_center;
    bodyVelocityToGeometric(desired_body_velocity, desired_center, &desired_omega);
    bodyVelocityToGeometric(present_body_velocity, present_center, &present_omega);

    Eigen::Vector3d body_velocity;

    if( desired_center(2) == 0.0 ||
        present_center(2) == 0.0 )
    {
        double present_angle = atan2(present_center(1), present_center(0));
        double desired_angle = atan2(desired_center(1), desired_center(0));
        double interpolated_angle = a*present_angle + (1.0-a)*desired_angle;
        body_velocity << cos(interpolated_angle), sin(interpolated_angle), 0.0;

        double present_speed = present_body_velocity.head(2).norm();
        double desired_speed = desired_body_velocity.head(2).norm();
        double interpolated_speed = a*present_speed + (1.0-a)*desired_speed;
        body_velocity *= interpolated_speed;

        body_velocity(2) = a*present_omega + (1.0-a)*desired_omega;
    }
    else
    {
        Eigen::Vector3d interpolated_center = a*present_center + (1.0-a)*desired_center;
        double interpolated_omega = a*present_omega + (1.0-a)*desired_omega;
        body_velocity = geometricToBody(interpolated_center, interpolated_omega);
    }
    return body_velocity;
}


Eigen::Vector3d NumericalLimit(
        const Eigen::Vector2d &stern_position,
        const Eigen::Vector2d &starboard_position,
        const Eigen::Vector2d &port_position,
        const double dtheta,
        const double min_step,
        const double tolerance,
        double stern_angle, double stern_velocity,
        double starboard_angle, double starboard_velocity,
        double port_angle, double port_velocity,
        const Eigen::Vector3d &desired_body_velocity
        )
{
    Eigen::Vector3d present_center, present_body_velocity;
    double present_omega;
    double error = podParamtersToGeometric(
        stern_position,
        starboard_position,
        port_position,
        stern_angle, stern_velocity,
        starboard_angle, starboard_velocity,
        port_angle, port_velocity,
        present_center,
        &present_omega
        );

    present_body_velocity = geometricToBody(present_center, present_omega);

    if((present_body_velocity - desired_body_velocity).norm()<0.01)
        return desired_body_velocity;

    double desired_speed = desired_body_velocity.head(2).norm();
    if(desired_speed<MIN_SPEED && desired_body_velocity(2)<MIN_OMEGA)
        return desired_body_velocity;

    double desired_stern_angle, desired_starboard_angle, desired_port_angle;
    double desired_stern_speed, desired_starboard_speed, desired_port_speed;
    computePod(stern_position, desired_body_velocity, &desired_stern_angle, &desired_stern_speed);
    computePod(starboard_position, desired_body_velocity, &desired_starboard_angle, &desired_starboard_speed);
    computePod(port_position, desired_body_velocity, &desired_port_angle, &desired_port_speed);

    flipVelocity(&stern_angle, &stern_velocity);
    flipVelocity(&starboard_angle, &starboard_velocity);
    flipVelocity(&port_angle, &port_velocity);

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
    double err=fabs(limiting_dtheta) - dtheta;
    double a=1.0;
    std::cerr << "a: " << a << " err: " << err << std::endl;
    Eigen::Vector3d achievable_body_velocity=geometricInterpolate(a, desired_body_velocity, present_body_velocity) ;
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
        achievable_body_velocity = geometricInterpolate(a, desired_body_velocity, present_body_velocity);
        //std::cerr << "achievable: " << achievable_body_velocity.transpose() << std::endl;
        computePod(limiting_position, achievable_body_velocity, &limiting_theta, &limiting_speed);
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
