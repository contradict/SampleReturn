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

template
void
UKF::ScaledUKF<PoseUKF::PoseState>::correct(const struct PoseUKF::VisualOdometryMeasurement& measured,
                                            const std::vector<Eigen::MatrixXd>& measurement_covs
                                           );


namespace PoseUKF {

const double IMUOrientationMeasurement::littleg=9.8066;

std::ostream &
operator<<(std::ostream &out, const PoseState& st)
{
    double r, p, y;
    tf::Quaternion q;
    tf::quaternionEigenToTF(st.Orientation.unit_quaternion(), q);
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    out << "Position      (" << st.Position.transpose() << ")\n";
    out << "Velocity      (" << st.Velocity.transpose() << ")\n";
    out << "Acceleration  (" << st.Acceleration.transpose() << ")\n";
    out << "Orient(r,p,y) (" << r << ", " << p << ", " << y << ")\n";
    out << "Omega         (" << st.Omega.transpose() << ")\n";
    out << "GyroBias      (" << st.GyroBias.transpose() << ")\n";
    out << "AccelBias     (" << st.AccelBias.transpose() << ")\n";
    return out;
}

std::ostream &
operator<<(std::ostream &out, const IMUOrientationMeasurement& m)
{
    out << "IMU accel: (" << m.acceleration.transpose() << ")" << std::endl;
    out << "IMU omega: (" << m.omega.transpose() << ")" << std::endl;
    return out;
}

std::ostream &
operator<<(std::ostream &out, const WheelOdometryMeasurement& m)
{
    out << "odom delta yaw (" << m.delta_yaw << ")" << std::endl;
    out << "odom yaw rate  (" << m.yaw_rate << ")" << std::endl;
    out << "odom delta_pos (" << m.delta_pos.transpose() << ")" << std::endl;
    out << "odom velocity  (" << m.velocity.transpose() << ")" << std::endl;
    return out;
}


std::ostream &
operator<<(std::ostream &out, const VisualOdometryMeasurement& m)
{
    out << "vo delta_pos      (" << m.delta_pos.transpose() << ")" << std::endl;
    out << "vo velocity       (" << m.velocity.transpose() << ")" << std::endl;
    double r, p, y;
    tf::Quaternion q;
    tf::quaternionEigenToTF(m.delta_orientation.unit_quaternion(), q);
    tf::Matrix3x3 R(q);
    R.getRPY(r, p, y);
    out << "vo delta_o(r,p,y) (" << r << ", " << p << ", " << y << ")\n";
    out << "vo omega          (" << m.omega.transpose() << ")" << std::endl;
    return out;
}
}
