#include <pose_ukf/pose_ukf.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

template
void
UKF::ScaledUKF<PoseUKF::PoseState>::correct(const struct PoseUKF::IMUOrientationMeasurement& measured);

template
void
UKF::ScaledUKF<PoseUKF::PoseState>::correct(const struct PoseUKF::WheelOdometryMeasurement& measured);


namespace PoseUKF {

std::ostream &
operator<<(std::ostream &out, const PoseState& st)
{
    double r, p, y;
    tf::Quaternion q;
    tf::quaternionEigenToTF(st.Orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    out << "Position    (" << st.Position.transpose() << ")\n";
    out << "Velocity    (" << st.Velocity.transpose() << ")\n";
    out << "Orientation (" << r << ", " << p << ", " << y << ")\n";
    out << "Omega       (" << st.Omega.transpose() << ")\n";
    out << "GyroBias    (" << st.GyroBias.transpose() << ")\n";
    out << "AccelBias   (" << st.AccelBias.transpose() << ")\n";
}

std::ostream &
operator<<(std::ostream &out, const IMUOrientationMeasurement& m)
{
    out << "IMU acceleration: (" << m.acceleration.transpose() << ")" << std::endl;
    out << "IMU omega: (" << m.omega.transpose() << ")" << std::endl;
    return out;
}

std::ostream &
operator<<(std::ostream &out, const WheelOdometryMeasurement& m)
{
    out << "wheel positions:\n" << m.wheel_positions << std::endl;
    out << "wheel motions:\n" << m.wheel_motions << std::endl;
    out << "wheel Velocities: (" << m.wheel_velocities.transpose() << ")" << std::endl;
    return out;
}

Eigen::MatrixXd
PoseUKF::process_noise(double dt) const
{
    Eigen::MatrixXd noise(ndim(), ndim());
    noise.setZero();

    // position, velocity, orientation, omega, gyro_bias, accel_bias
    noise.block<3,3>(0,0).diagonal() = (dt*sigma_position).cwiseProduct(dt*sigma_position);
    noise.block<3,3>(0,3).diagonal() = (dt*dt*sigma_velocity/2.).cwiseProduct(dt*dt*sigma_velocity/2.);
    noise.block<3,3>(3,0).diagonal() = (dt*dt*sigma_velocity/2.).cwiseProduct(dt*dt*sigma_velocity/2.);
    noise.block<3,3>(3,3).diagonal() = (dt*sigma_velocity).cwiseProduct(dt*sigma_velocity);
    noise.block<3,3>(6,6).diagonal() = (dt*sigma_orientation).cwiseProduct(dt*sigma_orientation);
    noise.block<3,3>(6,9).diagonal() = (dt*dt*sigma_omega/2.).cwiseProduct(dt*dt*sigma_omega/2.);
    noise.block<3,3>(9,6).diagonal() = (dt*dt*sigma_omega/2.).cwiseProduct(dt*dt*sigma_omega/2.);
    noise.block<3,3>(9,9).diagonal() = (dt*sigma_omega).cwiseProduct(dt*sigma_omega);
    noise.block<3,3>(12,12).diagonal() = (dt*sigma_gyro_bias).cwiseProduct(dt*sigma_gyro_bias);
    noise.block<3,3>(15,15).diagonal() = (dt*sigma_accel_bias).cwiseProduct(dt*sigma_accel_bias);

    return noise;
}
}
