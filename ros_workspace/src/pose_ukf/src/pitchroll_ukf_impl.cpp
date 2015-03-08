#include <pose_ukf/pitchroll_ukf.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

template
void
UKF::ScaledUKF<PitchRollUKF::PitchRollState>::correct(const struct PitchRollUKF::IMUOrientationMeasurement& measured,
                                                      const std::vector<Eigen::MatrixXd>& measurement_covs
                                                     );

namespace PitchRollUKF {

std::ostream &
operator<<(std::ostream &out, const PitchRollState& st)
{
    double r, p, y;
    tf::Quaternion q;
    tf::quaternionEigenToTF(st.Orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    out << "PitchRoll State:" << std::endl;
    out << "\tOrientation (" << r << ", " << p << ", " << y << ")\n";
    out << "\tOmega       (" << st.Omega.transpose() << ")\n";
    out << "\tGyroBias    (" << st.GyroBias.transpose() << ")\n";
    out << "\tAccelBias   (" << st.AccelBias.transpose() << ")\n";
    return out;
}

std::ostream &
operator<<(std::ostream &out, const IMUOrientationMeasurement& m)
{
    out << "IMU measurement:\n\tacceleration: (" << m.acceleration.transpose() << ")" << std::endl;
    out << "\tomega: (" << m.omega.transpose() << ")" << std::endl;
    return out;
}

Eigen::MatrixXd
PitchRollUKF::process_noise(double dt) const
{
    Eigen::MatrixXd noise(ndim(), ndim());
    noise.setZero();

    // position, velocity, orientation, omega, gyro_bias, accel_bias
    noise.block<2,2>(0,0).diagonal() = (dt*sigma_orientation).cwiseProduct(dt*sigma_orientation);
    noise.block<2,2>(0,2).diagonal() = (dt*dt*sigma_omega/2.).cwiseProduct(dt*dt*sigma_omega/2.);
    noise.block<2,2>(2,0).diagonal() = (dt*dt*sigma_omega/2.).cwiseProduct(dt*dt*sigma_omega/2.);
    noise.block<2,2>(2,2).diagonal() = (dt*sigma_omega).cwiseProduct(dt*sigma_omega);
    noise.block<2,2>(4,4).diagonal() = (dt*sigma_gyro_bias).cwiseProduct(dt*sigma_gyro_bias);
    noise.block<3,3>(6,6).diagonal() = (dt*sigma_accel_bias).cwiseProduct(dt*sigma_accel_bias);

    return noise;
}
}
