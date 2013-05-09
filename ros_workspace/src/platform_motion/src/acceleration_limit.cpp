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
    pod_velocity(0) = body_velocity(0) - body_velocity(2)*position(0);
    pod_velocity(1) = body_velocity(1) + body_velocity(2)*position(1);
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

}
