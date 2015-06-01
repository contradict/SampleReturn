#include <pose_ukf/sunyaw_ukf.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

template
void
UKF::ScaledUKF<SunYawUKF::SunYawState>::correct(const struct SunYawUKF::IMUOrientationMeasurement& measured,
                                                      const std::vector<Eigen::MatrixXd>& measurement_covs
                                                     );
template
void
UKF::ScaledUKF<SunYawUKF::SunYawState>::correct(const struct SunYawUKF::SunSensorMeasurement& measured,
                                                      const std::vector<Eigen::MatrixXd>& measurement_covs
                                                     );

const double SunYawUKF::IMUOrientationMeasurement::littleg=9.8066;

namespace SunYawUKF {

std::ostream &
operator<<(std::ostream &out, const SunYawState& st)
{
    double r, p, y;
    tf::Quaternion q;
    tf::quaternionEigenToTF(st.Orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    out << "SunYaw State:" << std::endl;
    out << "\tOrientation (" << r << ", " << p << ", " << y << ")\n";
    out << "\tOmega       (" << st.Omega.transpose() << ")\n";
    out << "\tGyroBias    (" << st.GyroBias.transpose() << ")\n";
    return out;
}

std::ostream &
operator<<(std::ostream &out, const IMUOrientationMeasurement& m)
{
    out << "IMU measurement:\n\tacceleration: (" << m.acceleration.transpose() << ")" << std::endl;
    out << "\tomega: (" << m.omega.transpose() << ")" << std::endl;
    return out;
}

std::ostream &
operator<<(std::ostream &out, const SunSensorMeasurement& m)
{
    out << "Sun measurement:\n\treference: (" << m.reference.transpose() << ")" << std::endl;
    out << "\tmeasurement: (" << m.measurement.transpose() << ")" << std::endl;
    return out;
}


}
