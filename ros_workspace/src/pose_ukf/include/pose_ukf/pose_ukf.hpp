#include <pose_ukf/ukf.hpp>
#include <ostream>
#include <sophus/so3.hpp>
#include <pose_ukf/rotation_average.h>

namespace PoseUKF {

struct PoseState
{
    Eigen::Vector2d Position;
    Eigen::Vector2d Velocity;
    Eigen::Vector2d Acceleration;
    Sophus::SO3d Orientation;
    Eigen::Vector3d Omega;
    Eigen::Vector3d GyroBias;
    Eigen::Vector3d AccelBias;
    PoseState()
    {
        Position.setZero();
        Velocity.setZero();
        Acceleration.setZero();
        Orientation.setQuaternion(Eigen::Quaterniond::Identity());
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
    };
    struct PoseState
    boxplus(const Eigen::VectorXd& offset) const
    {
        struct PoseState out;
        out.Position = Position + offset.segment<2>(0);
        out.Velocity = Velocity + offset.segment<2>(2);
        out.Acceleration = Acceleration + offset.segment<2>(4);
        Sophus::SO3d rot = Sophus::SO3d::exp(offset.segment<3>(6));
        out.Orientation = rot*Orientation;
        out.Omega = Omega + offset.segment<3>(9);
        out.GyroBias = GyroBias + offset.segment<3>(12);
        out.AccelBias = AccelBias + offset.segment<3>(15);
        return out;
    };
    Eigen::VectorXd
    boxminus(const struct PoseState& other) const
    {
        Eigen::VectorXd out(ndim());

        out.segment<2>(0) = Position - other.Position;
        out.segment<2>(2) = Velocity - other.Velocity;
        out.segment<2>(4) = Acceleration - other.Acceleration;
        out.segment<3>(6) = Sophus::SO3d::log(Orientation*other.Orientation.inverse());
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
        Eigen::Vector3d deltapos;
        deltapos.segment<2>(0) = (dt*Velocity + dt*dt*Acceleration/2.);
        deltapos(2) = 0;
        out.Position = Position + (Orientation*deltapos).segment<2>(0) + Chinu.segment<2>(0);
        out.Velocity = Velocity + dt*Acceleration + Chinu.segment<2>(2);
        out.Acceleration = Acceleration + Chinu.segment<2>(4);
        Eigen::Vector3d delta_orientation = Omega*dt + Chinu.segment<3>(6);
        out.Orientation = Orientation*Sophus::SO3d::exp(delta_orientation);
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
        Acceleration.setZero();
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
        RotationAverage ra;
        for(auto && t: zip_range(weights, Chistate))
        {
            double w=t.get<0>();
            const struct PoseState& pt = t.get<1>();
            Position += w*pt.Position;
            Velocity += w*pt.Velocity;
            Acceleration += w*pt.Acceleration;
            ra(w, pt.Orientation);
            Omega += w*pt.Omega;
            GyroBias += w*pt.GyroBias;
            AccelBias += w*pt.AccelBias;
        }
        double error = ra.geodesic(&Orientation);
        if(error<0)
        {
            Orientation = Chistate.front().Orientation;
        }
    };

    ssize_t ndim(void) const {return 3*2 + 4*3;};

    friend std::ostream& operator<<(std::ostream &, const PoseState &);
};

struct WheelOdometryMeasurement
{
    double delta_yaw;
    double yaw_rate;
    Eigen::Vector2d delta_pos;
    Eigen::Vector2d velocity;
    struct PoseState last_state;

    WheelOdometryMeasurement() :
        delta_yaw(0.0),
        yaw_rate(0.0)
    {
        delta_pos.setZero();
        velocity.setZero();
    };

    Eigen::VectorXd boxminus(const struct WheelOdometryMeasurement& other) const
    {
        Eigen::VectorXd diff(ndim());
        diff(0) = delta_yaw - other.delta_yaw;
        diff(1) = yaw_rate - other.yaw_rate;
        diff.segment<2>(2) = delta_pos - other.delta_pos;
        diff.segment<2>(4) = velocity - other.velocity;
        return diff;
    };

    void mean(const std::vector<double>& weights,
            const std::vector<struct WheelOdometryMeasurement>& Chimeas)
    {
        delta_yaw = 0;
        yaw_rate = 0;
        delta_pos.setZero();
        velocity.setZero();
        for(const auto &&t: zip_range(weights, Chimeas))
        {
            double w = t.get<0>();
            struct WheelOdometryMeasurement m = t.get<1>();
            delta_yaw += w*m.delta_yaw;
            yaw_rate += w*m.yaw_rate;
            delta_pos += w*m.delta_pos;
            velocity += w*m.velocity;
        }
    };

    struct WheelOdometryMeasurement
    measure(const struct PoseState& st, const Eigen::VectorXd& noise) const
    {
        struct WheelOdometryMeasurement m;
        Sophus::SO3d::Tangent delta_orientation;
        delta_orientation = Sophus::SO3d::log(st.Orientation*last_state.Orientation.inverse());
        m.delta_yaw = delta_orientation(2) + noise(0);
        m.yaw_rate = st.Omega(2) + noise(1);
        m.delta_pos = st.Position - last_state.Position + noise.segment<2>(2);
        m.velocity = st.Velocity + noise.segment<2>(4);
        return m;
    };

    ssize_t ndim(void) const { return 6; };
    friend std::ostream& operator<<(std::ostream &, const WheelOdometryMeasurement &);
};

struct IMUOrientationMeasurement
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d omega;

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
        m.acceleration = st.Orientation.inverse() * gravity + st.AccelBias;
        m.acceleration.segment<2>(0) += st.Acceleration;
        m.acceleration += noise.segment<3>(0);
        m.omega = st.Omega + st.GyroBias + noise.segment<3>(3);
        return m;
    };

    ssize_t ndim(void) const { return 6; };
    static const double littleg;
    friend std::ostream& operator<<(std::ostream &, const IMUOrientationMeasurement &);
};

class PoseUKF : public UKF::ScaledUKF<struct PoseState>
{
    public:
    PoseUKF(double alpha=0.1, double beta=2.0, double kappa=0.0) :
        ::UKF::ScaledUKF<struct PoseState>(alpha, beta, kappa)
    {
    };

    ssize_t ndim(void) const {return state().ndim(); };
};

};
