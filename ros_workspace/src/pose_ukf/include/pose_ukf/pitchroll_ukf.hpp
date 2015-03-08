#include <pose_ukf/ukf.hpp>
#include <ostream>

namespace PitchRollUKF {

Eigen::Vector3d
log(const Eigen::Quaterniond& other)
{
    double squared_n
        = other.vec().squaredNorm();
    double n = std::sqrt(squared_n);
    double w = other.w();

    double two_atan_nbyw_by_n;

    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    if (n < 1e-10) {
        double squared_w = w*w;
        two_atan_nbyw_by_n = 2.0 / w
            - 2.0*(squared_n)/(w*squared_w);
    } else {
        if (std::abs(w)<1e-10) {
            if (w > 0.0) {
                two_atan_nbyw_by_n = M_PI/n;
            } else {
                two_atan_nbyw_by_n = -M_PI/n;
            }
        }else{
            two_atan_nbyw_by_n = 2.0 * atan(n/w) / n;
        }
    }

    return two_atan_nbyw_by_n * other.vec();
}

Eigen::Quaterniond
exp(const Eigen::Vector3d& omega)
{
    double theta_sq = omega.squaredNorm();
    double theta = std::sqrt(theta_sq);
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor;;
    if(theta<1e-10) {
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = (0.5)
            - (1.0/48.0)*theta_sq
            + (1.0/3840.0)*theta_po4;
        real_factor = (1.0)
            - (0.5)*theta_sq +
            (1.0/384.0)*theta_po4;
    } else {
        double sin_half_theta = std::sin(half_theta);
        imag_factor = sin_half_theta/(theta);
        real_factor = std::cos(half_theta);
    }

    return Eigen::Quaterniond(real_factor,
            imag_factor*omega.x(),
            imag_factor*omega.y(),
            imag_factor*omega.z());
}

struct PitchRollState : public UKF::State<struct PitchRollState> {
    Eigen::Quaterniond Orientation;
    Eigen::Vector2d Omega;
    Eigen::Vector2d GyroBias;
    Eigen::Vector3d AccelBias;
    PitchRollState()
    {
        Orientation.setIdentity();
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
    };
    struct PitchRollState
    boxplus(const Eigen::VectorXd& offset) const
    {
        struct PitchRollState out;
        Eigen::Vector3d omega(offset(0), offset(1), 0.0);
        Eigen::Quaterniond rot=exp(omega);
        out.Orientation = rot*Orientation;
        out.Omega = Omega + offset.segment<2>(2);
        out.GyroBias = GyroBias + offset.segment<2>(4);
        out.AccelBias = AccelBias + offset.segment<3>(6);
        return out;
    };
    Eigen::VectorXd
    boxminus(const struct PitchRollState& other) const
    {
        Eigen::VectorXd out(ndim());

        Eigen::Quaterniond qdiff = Orientation*other.Orientation.inverse();
        out.segment<3>(0) = log(qdiff);
        out.segment<2>(2) = Omega - other.Omega;
        out.segment<2>(4) = GyroBias - other.GyroBias;
        out.segment<3>(6) = AccelBias - other.AccelBias;

        return out;
    };
    struct PitchRollState
    advance(double dt,
            const Eigen::VectorXd &Chinu) const
    {
        struct PitchRollState out;
        Eigen::Vector3d o;
        o.segment<2>(0) = Omega*dt + Chinu.segment<2>(0);
        o(2) = 0.0;
        Eigen::Quaterniond rot=exp(o);
        out.Orientation = rot*Orientation;
        out.Omega = Omega + Chinu.segment<2>(2);
        out.GyroBias = GyroBias + Chinu.segment<2>(4);
        out.AccelBias = AccelBias + Chinu.segment<3>(6);
        return out;
    };
    void mean(const std::vector<double> &weights,
              const std::vector<struct PitchRollState> &Chistate)
    {
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
        Eigen::Matrix4d Q;
        Q.setZero();
        double max_angle=0;
        auto rest = Chistate.begin();
        rest++;
        for(auto && t: zip_range(weights, Chistate))
        {
            double w=t.get<0>();
            const struct PitchRollState& pt = t.get<1>();
            Q += w*(4.0*pt.Orientation.coeffs() * pt.Orientation.coeffs().transpose() - Eigen::Matrix4d::Identity());
            max_angle = std::accumulate(rest, Chistate.end(), max_angle,
                    [&pt](double max, const struct PitchRollState& other)
                    {
                    return std::max(max,
                        acos((pt.Orientation.inverse()*other.Orientation).w())*2.);
                    });
            Omega += w*pt.Omega;
            GyroBias += w*pt.GyroBias;
            AccelBias += w*pt.AccelBias;
        }
        if(max_angle<M_PI_2)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
            solver.compute(Q);
            Orientation.coeffs() = solver.eigenvectors().col(3);
            Orientation.z() = 0.0;
            Orientation.normalize();
        }
        else
        {
            Orientation = Chistate.front().Orientation;
        }
    };
    ssize_t ndim(void) const {return 3*2+3;};

    friend std::ostream& operator<<(std::ostream &, const PitchRollState &);
};

struct IMUOrientationMeasurement : UKF::Measurement<struct IMUOrientationMeasurement, struct PitchRollState>
{
    Eigen::Vector3d acceleration;
    Eigen::Vector2d omega;
    double delta_t;

    IMUOrientationMeasurement()
    {
        acceleration.setZero();
        omega.setZero();
    };

    Eigen::VectorXd boxminus(const struct IMUOrientationMeasurement& other) const
    {
        Eigen::VectorXd diff(ndim());
        diff.segment<3>(0) = acceleration - other.acceleration;
        diff.segment<2>(3) = omega - other.omega;
        return diff;
    };

    void mean(const std::vector<double>& weights,
            const std::vector<struct IMUOrientationMeasurement>& Chimeas)
    {
        acceleration.setZero();
        omega.setZero();
        for(const auto &&t: zip_range(weights, Chimeas))
        {
            double w = t.get<0>();
            struct IMUOrientationMeasurement m = t.get<1>();
            acceleration += w*m.acceleration;
            omega += w*m.omega;
        }
    };

    struct IMUOrientationMeasurement
    measure(const struct PitchRollState& st, const Eigen::VectorXd& noise) const
    {
        struct IMUOrientationMeasurement m;
        Eigen::Vector3d gravity(0, 0, -littleg);
        m.acceleration = (st.Orientation * gravity) + st.AccelBias + noise.segment<3>(0);
        m.omega = st.Omega + st.GyroBias + noise.segment<2>(3);
        return m;
    };

    ssize_t ndim(void) const { return 5; };
    constexpr static const double littleg=9.8066;
    friend std::ostream& operator<<(std::ostream &, const IMUOrientationMeasurement &);
};

class PitchRollUKF : public UKF::ScaledUKF<struct PitchRollState>
{
    Eigen::MatrixXd
    process_noise(double dt) const;

    public:

    Eigen::Vector2d sigma_orientation;
    Eigen::Vector2d sigma_omega;
    Eigen::Vector2d sigma_gyro_bias;
    Eigen::Vector3d sigma_accel_bias;

    PitchRollUKF(double alpha=0.1, double beta=2.0, double kappa=0.0) :
        ::UKF::ScaledUKF<struct PitchRollState>(alpha, beta, kappa)
    {
        sigma_orientation.setOnes();
        sigma_omega.setOnes();
        sigma_gyro_bias.setOnes();
        sigma_accel_bias.setOnes();
    };
};

}
