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
    if(*angle<-M_PI_2)
    {
        *angle += M_PI;
        *velocity *= -1;
    }
}

double deltaAngle(double a, double b)
{
    double delta = a-b;
    return std::min(std::min(fabs(delta), fabs(delta+M_PI)), fabs(delta-M_PI));
}

#define MIN_OMEGA 0.01
#define MIN_SPEED 0.05
void bodyVelocityToGeometric(
        const Eigen::Vector3d &body_velocity,
        Eigen::Vector3d &center,
        double *omega
        )
{
    double speed = body_velocity.head(2).norm();
    double radius = speed/body_velocity(2);
    Eigen::Vector3d zhat;
    zhat << 0,0,1;
    Eigen::Vector3d velocity3;
    velocity3.head(2) = body_velocity.head(2);
    velocity3(2) = 0.0;
    Eigen::Vector3d direction;
    if(velocity3.norm()>MIN_SPEED)
        direction = zhat.cross(velocity3).normalized();
    else
        direction << 0,0,0;
    if(fabs(body_velocity(2))>MIN_OMEGA)
    {
        center = radius*direction;
        center(2) = 1.0;
        *omega = body_velocity(2);
    }
    else
    {
        center = direction;
        center(2) = 0.0;
        *omega = speed;
        if(body_velocity(2)<0.0)
            *omega *= -1.0;
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
        Eigen::Vector3d &center,
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
        //std::cerr << "direction: " << direction << std::endl;
        error = (fabs(stern_angle - direction) + fabs(starboard_angle-direction) + fabs(port_angle-direction))/3.0;
        //std::cerr << "error: " << error << std::endl;
        double velocity = (stern_velocity + starboard_velocity + port_velocity)/3.0;
        //std::cerr << "velocity: " << velocity << std::endl;
        center << -sin(direction), cos(direction), 0;
        //std::cerr << "center: " << center.transpose() << std::endl;
        *omega = velocity;
    }
    else
    {
        Eigen::Vector3d stern_axle_direction, starboard_axle_direction, port_axle_direction;
        // ax+by+c=0
        // y=sin(a)/cos(a)*x
        // a=-sin(a), b=cos(a), c=0
        stern_axle_direction << cos(stern_angle), sin(stern_angle), 0;
        if(stern_velocity<0.0)
        {
            stern_axle_direction *= -1.0;
        }
        starboard_axle_direction << cos(starboard_angle), sin(starboard_angle), 0;
        if(starboard_velocity<0.0)
        {
            starboard_axle_direction *= -1.0;
        }
        port_axle_direction << cos(port_angle), sin(port_angle), 0;
        if(port_velocity<0.0)
        {
            port_axle_direction *= -1.0;
        }

        // line through pod position in direction axle is pointing.
        stern_axle_direction(2) = -stern_axle_direction.head(2).dot(stern_position);
        starboard_axle_direction(2) = -starboard_axle_direction.head(2).dot(starboard_position);
        port_axle_direction(2) = -port_axle_direction.head(2).dot(port_position);

        std::cerr << "stern_position: " << stern_position.transpose() << " starboard_position: " << starboard_position.transpose() << " port_position: " << port_position.transpose() << std::endl;
        std::cerr << "stern_axle_direction: " << stern_axle_direction.transpose() << " starboard_axle_direction: " << starboard_axle_direction.transpose() << " port_axle_direction: " << port_axle_direction.transpose() << std::endl;

        Eigen::Vector3d i1, i2, i3;
        i1 = stern_axle_direction.cross(starboard_axle_direction);
        i2 = stern_axle_direction.cross(port_axle_direction);
        i3 = starboard_axle_direction.cross(port_axle_direction);

        std::cerr << "i1: " << i1.transpose() << " i2: " << i2.transpose() << " i3: " << i3.transpose() << std::endl;

        int cnt=0;
        if( i1.norm()>PARALLEL_THRESHOLD )
        {
            i1 /= i1(2);
            center += i1;
            cnt++;
        }
        if( i2.norm()>PARALLEL_THRESHOLD )
        {
            i2 /= i2(2);
            center += i2;
            cnt++;
        }
        if( i3.norm()>PARALLEL_THRESHOLD )
        {
            i3 /= i3(2);
            center += i3;
            cnt++;
        }
        center /= cnt;
        if( i1.norm()>PARALLEL_THRESHOLD )
        {
            error += (i1-center).norm();
        }
        if( i2.norm()>PARALLEL_THRESHOLD )
        {
            error += (i2-center).norm();
        }
        if( i3.norm()>PARALLEL_THRESHOLD )
        {
            error += (i3-center).norm();
        }
        error /= cnt;

        cnt = 0;
        Eigen::Vector2d stern_radius, starboard_radius, port_radius;
        stern_radius = (stern_position - center.head(2));
        starboard_radius = (starboard_position - center.head(2));
        port_radius = (port_position - center.head(2));
        std::cerr << "stern_radius: " << stern_radius.transpose() << " starboard_radius: " << starboard_radius.transpose() << " port_radius: " << port_radius.transpose() << std::endl;
        *omega = 0.0;
        //(0,0,omega) = (rx, ry, 0) X (vx, vy, 0) / |r|^2
        // omega = (rx*vy - ry*vx)/|r|^2
        double n=port_radius.norm();
        if( n > RADIUS_THRESHOLD )
        {
            *omega += port_velocity*(port_radius(0)*sin(port_angle) - port_radius(1)*cos(port_angle))/(n*n);
            cnt++;
            std::cerr << "port omega: " << *omega << std::endl;
        }
        n=starboard_radius.norm();
        if( n > RADIUS_THRESHOLD )
        {
            *omega += starboard_velocity*(starboard_radius(0)*sin(starboard_angle) - starboard_radius(1)*cos(starboard_angle))/(n*n);
            cnt++;
            std::cerr << "starboard omega: " << *omega << std::endl;
        }
        n=stern_radius.norm();
        if( n > RADIUS_THRESHOLD )
        {
            *omega += stern_velocity*(stern_radius(0)*sin(stern_angle) - stern_radius(1)*cos(stern_angle))/(n*n);
            cnt++;
            std::cerr << "stern omega: " << *omega << std::endl;
        }
        *omega /= cnt;
    }

    return error;

}

void geometricToPod(
        const Eigen::Vector2d &position,
        double *pod_angle, double *pod_velocity,
        Eigen::Vector3d &center,
        double omega
        )
{
    if(center(2) == 0.0 )
    {
        *pod_angle = atan2(-center(0), center(1));
        *pod_velocity = omega;
    }
    else
    {
        Eigen::Vector2d pod_vector = (center.head(2) - position);
        double pod_radius = pod_vector.norm();
        *pod_velocity = pod_radius*omega;
        *pod_angle = atan2(-pod_vector(0), pod_vector(1));
    }
    flipVelocity(pod_angle, pod_velocity);
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
    double speed = body_velocity.head(2).norm();
    if(speed<MIN_SPEED)
        body_velocity << 0.0, 0.0, 0.0;
    else
        body_velocity.head(2)/=speed;
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

void geometricInterpolate(
        double a,
        const Eigen::Vector3d desired_center,
        const double desired_omega,
        const Eigen::Vector3d present_center,
        const double present_omega,
        Eigen::Vector3d &interpolated_center,
        double *interpolated_omega
        )
{
    *interpolated_omega = a*desired_omega + (1.0-a)*present_omega;

    if( desired_center(2) == 0.0 ||
        present_center(2) == 0.0 )
    {
        double present_angle = atan2(present_center(1), present_center(0));
        double desired_angle = atan2(desired_center(1), desired_center(0));
        if( fabs(present_angle - desired_angle) > M_PI)
        {
            if(present_angle<0)
                present_angle += 2*M_PI;
            if(desired_angle<0)
                desired_angle += 2*M_PI;
        }
        double interpolated_angle = a*desired_angle + (1.0-a)*present_angle;
        if(interpolated_angle>M_PI)
            interpolated_angle -= 2*M_PI;

        std::cerr << "present angle: " << present_angle << " desired_angle: " << desired_angle << " interpolated_angle: " << interpolated_angle << std::endl;
        interpolated_center << cos(interpolated_angle), sin(interpolated_angle), 0.0;

    }
    else
    {
        interpolated_center = a*desired_center + (1.0-a)*present_center;
    }
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
        const Eigen::Vector3d &desired_body_velocity,
        Eigen::Vector3d &present_center, double *present_omega,
        Eigen::Vector3d &desired_center, double *desired_omega,
        Eigen::Vector3d &interpolated_center, double *interpolated_omega
        )
{
    flipVelocity(&stern_angle, &stern_velocity);
    flipVelocity(&starboard_angle, &starboard_velocity);
    flipVelocity(&port_angle, &port_velocity);
    double error = podParamtersToGeometric(
        stern_position,
        starboard_position,
        port_position,
        stern_angle, stern_velocity,
        starboard_angle, starboard_velocity,
        port_angle, port_velocity,
        present_center,
        present_omega
        );
    std::cerr << "present center: " << present_center.transpose() << " omega: " << *present_omega << std::endl;
    bodyVelocityToGeometric(desired_body_velocity, desired_center, desired_omega);
    std::cerr << "desired center: " << desired_center.transpose() << " omega: " << *desired_omega << std::endl;

    double desired_speed = desired_body_velocity.head(2).norm();
    if(desired_speed<MIN_SPEED && fabs(desired_body_velocity(2))<MIN_OMEGA)
        return desired_body_velocity;

    double desired_stern_angle, desired_starboard_angle, desired_port_angle;
    double desired_stern_speed, desired_starboard_speed, desired_port_speed;
    geometricToPod(
        stern_position,
        &desired_stern_angle, &desired_stern_speed,
        desired_center,
        *desired_omega
        );
    geometricToPod(
        starboard_position,
        &desired_starboard_angle, &desired_starboard_speed,
        desired_center,
        *desired_omega
        );
    geometricToPod(
        port_position,
        &desired_port_angle, &desired_port_speed,
        desired_center,
        *desired_omega
        );

    /*
    std::cerr << "present angle stern: " << stern_angle << " starboard: " << starboard_angle << " port: " << port_angle << std::endl;
    std::cerr << "present velocity stern: " << stern_velocity << " starboard: " << starboard_velocity << " port: " << port_velocity << std::endl;
    std::cerr << "desired angle stern: " << desired_stern_angle << " starboard: " << desired_starboard_angle << " port: " << desired_port_angle << std::endl;
    std::cerr << "desired velocity stern: " << desired_stern_speed << " starboard: " << desired_starboard_speed << " port: " << desired_port_speed << std::endl;
    */

    double stern_dtheta, starboard_dtheta, port_dtheta;
    stern_dtheta = deltaAngle(desired_stern_angle, stern_angle);
    starboard_dtheta = deltaAngle(desired_starboard_angle, starboard_angle);
    port_dtheta = deltaAngle(desired_port_angle, port_angle);

    Eigen::Vector2d limiting_position;
    double limiting_angle;
    double limiting_dtheta;
    if(fabs(stern_dtheta)>fabs(starboard_dtheta))
    {
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
    }
    else
    {
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
    }
    double err=limiting_dtheta - dtheta;
    double a=1.0;
    //std::cerr << "a: " << a << " err: " << err << " tolerance: " << tolerance << std::endl;
    geometricInterpolate(a, desired_center, *desired_omega, present_center, *present_omega, interpolated_center, interpolated_omega);
    int its=0;
    if(err>0)
        while( a>min_step )
        {
            its++;
            double limiting_theta, limiting_speed;
            if(err>0)
                a/=2;
            else
                a*=1.5;
            std::cerr << "a: " << a << std::endl;
            geometricInterpolate(a, desired_center, *desired_omega, present_center, *present_omega, interpolated_center, interpolated_omega);
            geometricToPod(
                    limiting_position,
                    &limiting_theta, &limiting_speed,
                    interpolated_center,
                    *interpolated_omega
                    );
            limiting_dtheta = deltaAngle(limiting_theta, limiting_angle);
            std::cerr << "limiting_theta: " << limiting_theta << " limiting_angle: " << limiting_angle << " limiting_dtheta: " << limiting_dtheta << " dtheta: " << dtheta << std::endl;
            err=limiting_dtheta - dtheta;
            std::cerr << "err: " << err << std::endl;
            if(err<0 && err>-tolerance)
                break;
        }
    std::cerr << "its: " << its << " a: " << a << std::endl;
    std::cerr << "interpolated center: " << interpolated_center.transpose() << " interpolated_omega: " << *interpolated_omega << std::endl;
    //std::cerr << "desired: " << desired_body_velocity.transpose() << std::endl;
    //std::cerr << "achievable: " << achievable_body_velocity.transpose() << std::endl;
    return geometricToBody(interpolated_center, *interpolated_omega);
}


}
