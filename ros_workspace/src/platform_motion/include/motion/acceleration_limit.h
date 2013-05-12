#include <Eigen/Dense>

namespace platform_motion {
Eigen::Vector3d VelocityBounds(
        Eigen::Vector3d body_velocity,
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double max_steering_accel
        );

Eigen::Vector3d SelectClosestVelocity(
        Eigen::Vector3d body_velocity,
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double max_steering_omega,
        Eigen::Vector3d deltav,
        Eigen::Vector3d desired_velocity);

Eigen::Vector3d SimpleLimit(
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double dtheta,
        double stern_angle,
        double starboard_angle,
        double port_angle,
        Eigen::Vector3d desired_body_velocity
        );

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
        );
}
