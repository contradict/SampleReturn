#include <Eigen/Dense>

namespace platform_motion {

Eigen::Vector3d geometricToBody(
        const Eigen::Vector3d &center,
        const double omega
        );

void bodyVelocityToGeometric(
        const Eigen::Vector3d &body_velocity,
        Eigen::Vector3d &center,
        double *omega
        );

void geometricToPod(
        const Eigen::Vector2d &position,
        double *pod_angle, double *pod_velocity,
        Eigen::Vector3d &center,
        double omega
        );

double podParamtersToGeometric(
        const Eigen::Vector2d &stern_position,
        const Eigen::Vector2d &starboard_position,
        const Eigen::Vector2d &port_position,
        double stern_angle, double stern_velocity,
        double starboard_angle, double starboard_velocity,
        double port_angle, double port_velocity,
        Eigen::Vector3d &center,
        double *omega
        );

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
        );
}
