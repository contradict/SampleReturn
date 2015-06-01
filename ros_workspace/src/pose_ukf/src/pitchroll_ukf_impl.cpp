#include <pose_ukf/pitchroll_ukf.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

template
void
UKF::ScaledUKF<PitchRollUKF::PitchRollState>::correct(const struct PitchRollUKF::IMUOrientationMeasurement& measured,
                                                      const std::vector<Eigen::MatrixXd>& measurement_covs
                                                     );

const double PitchRollUKF::IMUOrientationMeasurement::littleg=9.8066;

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
    //out << "\tAccelBias   (" << st.AccelBias.transpose() << ")\n";
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
