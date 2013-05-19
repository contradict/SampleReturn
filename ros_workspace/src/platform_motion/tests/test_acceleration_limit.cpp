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

TEST_F( AccelerationLimitsTest, bv2g )
{
    Eigen::Vector3d body_velocity;
    body_velocity << 1,0.2,0.0;
    Eigen::Vector3d center;
    double omega;

    platform_motion::bodyVelocityToGeometric(
        body_velocity,
        center,
        &omega
        );

    Eigen::Vector3d body_out = platform_motion::geometricToBody(center, omega);

    std::cout << "in: " << body_velocity.transpose() << " out: " << body_out.transpose() << std::endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
