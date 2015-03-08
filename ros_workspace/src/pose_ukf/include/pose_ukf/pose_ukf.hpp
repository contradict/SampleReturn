#include <pose_ukf/ukf.hpp>
#include <ostream>

namespace PoseUKF {

struct PoseState
{
    Eigen::Vector3d Position;
    Eigen::Vector3d Velocity;
    Eigen::Quaterniond Orientation;
    Eigen::Vector3d Omega;
    Eigen::Vector3d GyroBias;
    Eigen::Vector3d AccelBias;
    PoseState()
    {
        Position.resize(3);
        Position.setZero();
        Velocity.resize(3);
        Velocity.setZero();
        Orientation.setIdentity();
        Omega.resize(3);
        Omega.setZero();
        GyroBias.resize(3);
        GyroBias.setZero();
        AccelBias.resize(3);
        AccelBias.setZero();
    };
    struct PoseState
    boxplus(const Eigen::VectorXd& offset) const
    {
        struct PoseState out;
        out.Position = Position + offset.segment<3>(0);
        out.Velocity = Velocity + offset.segment<3>(3);
        Eigen::Quaterniond rot;
        rot.vec() = offset.segment<3>(6);
        double mag = rot.vec().norm();
        rot.vec() *= boost::math::sinc_pi(mag/2.0)/2.0;
        rot.w() = cos(mag/2.);
        out.Orientation = Orientation*rot;
        out.Omega = Omega + offset.segment<3>(9);
        out.GyroBias = GyroBias + offset.segment<3>(12);
        out.AccelBias = AccelBias + offset.segment<3>(15);
        return out;
    };
    Eigen::VectorXd
    boxminus(const struct PoseState& other) const
    {
        Eigen::VectorXd out(ndim());

        out.segment<3>(0) = Position - other.Position;
        out.segment<3>(3) = Velocity - other.Velocity;
        Eigen::Quaterniond qdiff = other.Orientation.inverse()*Orientation;
        double half_theta = acos(qdiff.w());
        out.segment<3>(6) = 2.0*qdiff.vec()/boost::math::sinc_pi(half_theta);
        out.segment<3>(9) = Omega - other.Omega;
        out.segment<3>(12) = GyroBias - other.GyroBias;
        out.segment<3>(15) = AccelBias - other.AccelBias;

        return out;
    };
    struct PoseState
    advance(double dt,
            const Eigen::VectorXd &Chinu) const
    {
        struct PoseState out;
        out.Position = Position + dt*Velocity + Chinu.segment<3>(0);
        out.Velocity = Velocity + Chinu.segment<3>(3);
        Eigen::Quaterniond rot;
        rot.vec() = Omega*dt + Chinu.segment<3>(6);
        double theta = rot.vec().norm();
        rot.w() = cos(theta/2.);
        rot.vec() *= boost::math::sinc_pi(theta/2.)/2.;
        out.Orientation = Orientation*rot;
        out.Omega = Omega + Chinu.segment<3>(9);
        out.GyroBias = GyroBias + Chinu.segment<3>(12);
        out.AccelBias = AccelBias + Chinu.segment<3>(15);
        return out;
    };
    void mean(const std::vector<double> &weights,
              const std::vector<struct PoseState> &Chistate)
    {
        Position.setZero();
        Velocity.setZero();
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
            const struct PoseState& pt = t.get<1>();
            Position += w*pt.Position;
            Velocity += w*pt.Velocity;
            Q += w*(4.0*pt.Orientation.coeffs() * pt.Orientation.coeffs().transpose() - Eigen::Matrix4d::Identity());
            max_angle = std::accumulate(rest, Chistate.end(), max_angle,
                    [&pt](double max, const struct PoseState& other)
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
        }
        else
        {
            Orientation = Chistate.front().Orientation;
        }
    };
    ssize_t ndim(void) const {return 6*3;};

    friend std::ostream& operator<<(std::ostream &, const PoseState &);
};

struct IMUOrientationMeasurement
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d omega;
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
        diff.segment<3>(3) = omega - other.omega;
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
    measure(const struct PoseState& st, const Eigen::VectorXd& noise) const
    {
        struct IMUOrientationMeasurement m;
        Eigen::Vector3d gravity(0, 0, -littleg);
        m.acceleration = st.Orientation * (gravity + st.AccelBias) + noise.segment<3>(0);
        m.omega = st.Omega + st.GyroBias + noise.segment<3>(3);
        return m;
    };

    ssize_t ndim(void) const { return 6; };
    constexpr static const double littleg=9.8066;
    friend std::ostream& operator<<(std::ostream &, const IMUOrientationMeasurement &);
};

struct WheelOdometryMeasurement
{
    Eigen::MatrixX2d wheel_motions;
    Eigen::MatrixX3d wheel_positions;
    Eigen::VectorXd wheel_velocities;
    struct PoseState base_state;
    double delta_t;

    WheelOdometryMeasurement()
    {
        wheel_motions.setZero();
        wheel_positions.setZero();
        wheel_velocities.setZero();
    };

    Eigen::VectorXd boxminus(const struct WheelOdometryMeasurement& other) const
    {
        Eigen::VectorXd diff(ndim());

        for(int i=0;i<wheel_motions.rows();i++)
        {
            diff(i*2) = wheel_motions(i,0) - other.wheel_motions(i,0);
            diff(i*2+1) = wheel_motions(i,1) - other.wheel_motions(i,1);
        }
        diff.segment(wheel_motions.rows()*2, wheel_velocities.rows()) = \
            wheel_velocities - other.wheel_velocities;

        return diff;
    };

    void
    mean(const std::vector<double>& weights,
         const std::vector<struct WheelOdometryMeasurement>& Chimeas)
    {
        wheel_positions = Chimeas.front().wheel_positions;
        int nmeas = Chimeas.front().wheel_motions.rows();
        wheel_motions.resize(nmeas, 2);
        wheel_motions.setZero();
        wheel_velocities.resize(nmeas);
        wheel_velocities.setZero();
        for(auto &&t : zip_range(weights, Chimeas))
        {
            double w = boost::get<0>(t);
            struct WheelOdometryMeasurement m = boost::get<1>(t);
            wheel_motions += w*m.wheel_motions;
            wheel_velocities += w*m.wheel_velocities;
        }
    }

    struct WheelOdometryMeasurement
    measure(const struct PoseState& st, const Eigen::VectorXd& noise) const
    {
        struct WheelOdometryMeasurement m;

        m.wheel_positions = wheel_positions;
        m.wheel_motions.resize(wheel_positions.rows(), 2);
        m.wheel_velocities.resize(wheel_positions.rows());
        for(int i=0; i<wheel_positions.rows(); i++)
        {
            m.wheel_motions.row(i) = \
                ((st.Position - base_state.Position) +
                st.Orientation*base_state.Orientation.inverse()*wheel_positions.row(i)).segment<2>(0) +
                noise.segment<2>(3*i);
            m.wheel_velocities(i) = \
                (st.Velocity + st.Omega.cross(wheel_positions.row(i))).norm() +
                noise(3*i+2);
        }

        return m;
    }

    ssize_t ndim(void) const
    {
        return 3*wheel_positions.rows();
    }

    friend std::ostream& operator<<(std::ostream &, const WheelOdometryMeasurement &);
};

class PoseUKF : public UKF::ScaledUKF<struct PoseState>
{
    Eigen::MatrixXd
    process_noise(double dt) const;

    public:

    Eigen::Vector3d sigma_position;
    Eigen::Vector3d sigma_velocity;
    Eigen::Vector3d sigma_orientation;
    Eigen::Vector3d sigma_omega;
    Eigen::Vector3d sigma_gyro_bias;
    Eigen::Vector3d sigma_accel_bias;

    PoseUKF(double alpha=0.1, double beta=2.0, double kappa=0.0) :
        ::UKF::ScaledUKF<struct PoseState>(alpha, beta, kappa)
    {
        sigma_position.setOnes();
        sigma_velocity.setOnes();
        sigma_orientation.setOnes();
        sigma_omega.setOnes();
        sigma_gyro_bias.setOnes();
        sigma_accel_bias.setOnes();
    };
};

};
