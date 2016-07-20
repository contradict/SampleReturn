#include <pose_ukf/pitchroll_ukf.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <pose_ukf/yaw_measurement.hpp>

template
void
UKF::ScaledUKF<PitchRollUKF::PitchRollState>::correct(const struct PitchRollUKF::IMUOrientationMeasurement& measured,
                                                      const std::vector<Eigen::MatrixXd>& measurement_covs
                                                     );

template
std::ostream &
PoseUKF::operator<<(std::ostream &out, const PoseUKF::YawMeasurement<PitchRollUKF::PitchRollState>& m);


namespace PitchRollUKF {

std::ostream &
operator<<(std::ostream &out, const PitchRollState& st)
{
    Eigen::Vector3d ypr = st.Orientation.unit_quaternion().toRotationMatrix().eulerAngles(2, 1, 0);
    out << "PitchRoll State:" << std::endl;
    out << "\tOrient ypr(" << ypr.transpose() << ")\n";
    out << "\tOrient   q(" << st.Orientation.unit_quaternion().coeffs().transpose() << ")\n";
    out << "\tOmega     (" << st.Omega.transpose() << ")\n";
    out << "\tGyroBias  (" << st.GyroBias.transpose() << ")\n";
    out << "\tAccelBias (" << st.AccelBias.transpose() << ")\n";
    return out;
}

std::ostream &
operator<<(std::ostream &out, const IMUOrientationMeasurement& m)
{
    out << "IMU measurement:\n\tacceleration: (" << m.acceleration.transpose() << ")" << std::endl;
    out << "\tomega: (" << m.omega.transpose() << ")" << std::endl;
    return out;
}

}
