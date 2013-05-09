#include <iostream>

#include "motion/acceleration_limit.h"

#include <gtest/gtest.h>

class AccelerationLimitsTest : public ::testing::Test
{
    public:
    Eigen::Vector2d stern_position;
    Eigen::Vector2d starboard_position;
    Eigen::Vector2d port_position;

    virtual void SetUp(void)
    {
        stern_position << -1,0;
        starboard_position << 0,1;
        port_position << 0,-1;
    }

};

TEST_F( AccelerationLimitsTest, simple )
{
    Eigen::Vector3d body_velocity;
    body_velocity << 1,0.2,0.0;

    double max_steering_accel=1.0;

    Eigen::Vector3d bounds;
    bounds = platform_motion::VelocityBounds(
            body_velocity,
            stern_position,
            starboard_position,
            port_position,
            max_steering_accel
            );
    std::cout << " bounds: " << bounds << std::endl;
}

TEST_F( AccelerationLimitsTest, circle )
{
    Eigen::Vector3d body_velocity;
    double max_steering_accel=1.0;
    for(double theta=-M_PI;theta<=M_PI;theta+=M_PI/8.0)
    {
        body_velocity << cos(theta),sin(theta),0.0;
        Eigen::Vector3d bounds;
        bounds = platform_motion::VelocityBounds(
                body_velocity,
                stern_position,
                starboard_position,
                port_position,
                max_steering_accel
                );
        std::cout << "theta: " << theta << " bounds: " << bounds.transpose() << std::endl;
    }
}

TEST_F( AccelerationLimitsTest, circleWithOmega )
{
    Eigen::Vector3d body_velocity;
    double max_steering_accel=1.0;
    for(double theta=-M_PI;theta<=M_PI;theta+=M_PI/8.0)
    {
        body_velocity << cos(theta),sin(theta),0.1;
        Eigen::Vector3d bounds;
        bounds = platform_motion::VelocityBounds(
                body_velocity,
                stern_position,
                starboard_position,
                port_position,
                max_steering_accel
                );
        std::cout << "theta: " << theta << " bounds: " << bounds.transpose() << std::endl;
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
