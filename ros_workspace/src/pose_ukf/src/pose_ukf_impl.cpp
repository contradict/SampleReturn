#include <pose_ukf/pose_ukf.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

template
void
UKF::ScaledUKF<PoseUKF::PoseState>::correct(const struct PoseUKF::IMUOrientationMeasurement& measured,
                                            const std::vector<Eigen::MatrixXd>& measurement_covs
                                           );

template
void
UKF::ScaledUKF<PoseUKF::PoseState>::correct(const struct PoseUKF::WheelOdometryMeasurement& measured,
                                            const std::vector<Eigen::MatrixXd>& measurement_covs
                                           );


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
    return out;
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


}
