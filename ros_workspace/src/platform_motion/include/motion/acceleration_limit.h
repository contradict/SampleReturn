#include <Eigen/Dense>

namespace platform_motion {
Eigen::Vector3d VelocityBounds(
        Eigen::Vector2d body_velocity,
        double body_omega,
        Eigen::Vector2d stern_position,
        Eigen::Vector2d starboard_position,
        Eigen::Vector2d port_position,
        double max_steering_accel
        );
}
