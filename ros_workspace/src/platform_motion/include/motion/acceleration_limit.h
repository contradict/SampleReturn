#include <Eigen/Dense>

namespace platform_motion {
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
        );
}
