#include "motion/acceleration_limit.h"
#include <iostream>

namespace platform_motion {

const double ACCEL_LIMIT_EPS=1e-3;

Eigen::Vector2d computePodVelocity(
        Eigen::Vector2d body_velocity,
        double body_omega,
        Eigen::Vector2d position
        )
{
    Eigen::Vector2d pod_velocity;
    pod_velocity(0) = body_velocity(0) - body_omega*position(0);
    pod_velocity(1) = body_velocity(1) + body_omega*position(1);
    return pod_velocity;
}

Eigen::Vector3d computeMatrixRow(
        Eigen::Vector2d body_velocity,
        double body_omega,
        Eigen::Vector2d position
        )
{
    Eigen::Vector2d V = computePodVelocity(body_velocity, body_omega, position);
    std::cout << "    " << "V      " << V << std::endl;
    double r = V.norm();
    if ( r < ACCEL_LIMIT_EPS )
    {
        return (Eigen::Vector3d() << NAN, NAN, NAN).finished();
    }
    double atan2_ = V(1)/(r + V(0));
    std::cout << "    " << "atan2_ " << atan2_ << std::endl;
    double coeff1 = 2./(1.+pow(atan2_,2.0));
    std::cout << "    " << "coeff1 " << coeff1 << std::endl;
    double A = coeff1*(1./(r + V(0)) - pow(atan2_,2.0)/r);
    std::cout << "    " << "A      " << A << std::endl;
    double B = coeff1*-1.0*V(1)/pow(r + V(0), 2.0)*(1.+V(0)/r);
    std::cout << "    " << "B      " << B << std::endl;
    double C = position(0);
    std::cout << "    " << "C      " << C << std::endl;
    double D = -position(1);
    std::cout << "    " << "D      " << D << std::endl;
    return (Eigen::Vector3d() << B, A, A*C+B*D).finished();
}

Eigen::Vector3d VelocityBounds(
        Eigen::Vector2d body_velocity,
        double body_omega,
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double max_steering_accel
        )
{
    Eigen::Matrix3d m;
    std::cout << "stern" << std::endl;
    m.row(0) = computeMatrixRow(body_velocity, body_omega, stern_position);
    if( isnan(m.row(0,0)) )
    {
    }
    std::cout << "  " << "row " << m.row(0) << std::endl;
    std::cout << "starboard" << std::endl;
    m.row(1) = computeMatrixRow(body_velocity, body_omega, starboard_position);
    std::cout << "  " << "row " << m.row(1) << std::endl;
    std::cout << "port" << std::endl;
    m.row(2) = computeMatrixRow(body_velocity, body_omega, port_position);
    std::cout << "  " << "row " << m.row(2) << std::endl;

    Eigen::Vector3d bounds;
    return m.fullPivLu().solve(Eigen::Vector3d::Constant(max_steering_accel));
}

}
