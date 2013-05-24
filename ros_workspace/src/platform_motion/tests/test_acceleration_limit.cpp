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
        starboard_position << 0,-1;
        port_position << 0,1;
    }

};

TEST_F( AccelerationLimitsTest, bv2g )
{
    Eigen::Vector3d body_velocity;
    Eigen::Vector3d body_out;
    for(double omega=-1.0; omega<=1.0; omega += 0.25)
        for(double theta=-M_PI; theta<=M_PI; theta += M_PI/8.)
        {
            body_velocity << cos(theta), sin(theta), omega;
            std::cout << "theta: " << theta << " omega: " << omega << " in: " << body_velocity.transpose() << std::endl;
            Eigen::Vector3d center;
            double omega;

            platform_motion::bodyVelocityToGeometric(
                body_velocity,
                center,
                &omega
                );

            std::cout << "center: " << center.transpose() << " omega: " << omega << std::endl;

            body_out = platform_motion::geometricToBody(center, omega);
            std::cout << "out: " << body_out.transpose() << " error: " << (body_velocity - body_out).norm() << std::endl;
            ASSERT_LT((body_velocity - body_out).norm(), 1e-10);
        }
}

TEST_F( AccelerationLimitsTest, pp2g )
{
    for(double w=0.0; w<=1.0; w+=1.0)
        for(double omega=-1.0; omega<=1.0; omega += 0.25)
            for(double theta=-M_PI; theta<=M_PI; theta += M_PI/8.)
            {
                Eigen::Vector3d center;
                center << cos(theta), sin(theta), w;
                std::cout << "center: " << center.transpose() << " omega: " << omega << std::endl;

                double stern_angle, starboard_angle, port_angle;
                double stern_velocity, starboard_velocity, port_velocity;
                platform_motion::geometricToPod(
                        stern_position,
                        &stern_angle, &stern_velocity,
                        center,
                        omega
                        );
                platform_motion::geometricToPod(
                        starboard_position,
                        &starboard_angle, &starboard_velocity,
                        center,
                        omega
                        );
                platform_motion::geometricToPod(
                        port_position,
                        &port_angle, &port_velocity,
                        center,
                        omega
                        );
                std::cout << "stern_angle: " << stern_angle << " starboard_angle: " << starboard_angle << " port_angle: " << port_angle << std::endl;
                std::cout << "stern_velocity: " << stern_velocity << " starboard_velocity: " << starboard_velocity << " port_velocity: " << port_velocity << std::endl;
                Eigen::Vector3d center_out;
                double omega_out;
                double err = platform_motion::podParamtersToGeometric(
                        stern_position,
                        starboard_position,
                        port_position,
                        stern_angle, stern_velocity,
                        starboard_angle, starboard_velocity,
                        port_angle, port_velocity,
                        center_out,
                        &omega_out
                        );
                std::cout << "center_out: " << center_out.transpose() << " omega_out: " << omega_out << std::endl;
                ASSERT_LT(fabs(err), 1e-10);
                if(center(2) != 0.0)
                    ASSERT_LT((center_out-center).norm(), 1e-10);
                else
                    ASSERT_LT((omega_out*center_out-omega*center).norm(), 1e-10);
                ASSERT_LT(fabs(pow(omega_out,2)-pow(omega,2)), 1e-10);
            }
}


TEST_F( AccelerationLimitsTest, g2p )
{
    Eigen::Vector3d center;
    double omega;
    double pod_angle, pod_velocity;
    Eigen::Vector2d position;

    position = starboard_position;

    center << 1, 0, 0;
    omega = 1.0;
    platform_motion::geometricToPod(
        position,
        &pod_angle, &pod_velocity,
        center,
        omega);
    std::cout << "center: " << center.transpose() << " omega: " << omega << std::endl;
    std::cout << "position: " << position.transpose() << " angle: " << pod_angle << " velocity: " << pod_velocity << std::endl;
    center << 1, 0.05, 0;
    omega = 1.0;
    platform_motion::geometricToPod(
        position,
        &pod_angle, &pod_velocity,
        center,
        omega);
    std::cout << "center: " << center.transpose() << " omega: " << omega << std::endl;
    std::cout << "position: " << position.transpose() << " angle: " << pod_angle << " velocity: " << pod_velocity << std::endl;
    center << 1, 0.1, 0;
    omega = 1.0;
    platform_motion::geometricToPod(
        position,
        &pod_angle, &pod_velocity,
        center,
        omega);
    std::cout << "center: " << center.transpose() << " omega: " << omega << std::endl;
    std::cout << "position: " << position.transpose() << " angle: " << pod_angle << " velocity: " << pod_velocity << std::endl;
}

TEST_F( AccelerationLimitsTest, nl )
{
    double dtheta = 0.05;
    double min_step = 0.001;
    double tolerance = 0.01;

    double stern_angle = 0.0;
    double starboard_angle = 0.0;
    double port_angle = 0.0;

    double stern_velocity = 0.01;
    double starboard_velocity = 0.01;
    double port_velocity = 0.01;

    Eigen::Vector3d desired_body_velocity;
    desired_body_velocity << -0.1, 1.0, 0.0;

    Eigen::Vector3d present_center, desired_center, interpolated_center;
    double present_omega, desired_omega, interpolated_omega;

    Eigen::Vector3d limited_velocity = platform_motion::NumericalLimit(
        stern_position,
        starboard_position,
        port_position,
        dtheta,
        min_step,
        tolerance,
        stern_angle, stern_velocity,
        starboard_angle, starboard_velocity,
        port_angle, port_velocity,
        desired_body_velocity,
        present_center, &present_omega,
        desired_center, &desired_omega,
        interpolated_center, &interpolated_omega
        );
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
