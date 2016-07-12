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

const double PoseState::littleg=9.8066;
const Eigen::Vector3d PoseState::gravity=Eigen::Vector3d(0, 0, -PoseState::littleg);

std::ostream &
operator<<(std::ostream &out, const PoseState& st)
{
    Eigen::Vector3d rpy = st.Orientation.unit_quaternion().toRotationMatrix().eulerAngles(0, 1, 2);
    out << "Position      (" << st.Position.transpose() << ")\n";
    out << "Velocity      (" << st.Velocity.transpose() << ")\n";
    out << "Acceleration  (" << st.Acceleration.transpose() << ")\n";
    out << "Orient(r,p,y) (" << rpy.transpose() << ")\n";
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
    out << "delta yaw: " << m.delta_yaw << std::endl;
    out << "yaw rate: " << m.yaw_rate << std::endl;
    out << "delta_pos: " << m.delta_pos.transpose() << std::endl;
    out << "velocity: " << m.velocity.transpose() << std::endl;
    return out;
}

std::ostream &
operator<<(std::ostream &out, const YawMeasurement& m)
{
    out << "yaw: " << m.yaw << std::endl;
    return out;
}

}
