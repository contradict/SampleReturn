#include <pose_ukf/ukf.hpp>
#include <ostream>
#include <tuple>
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
    Eigen::MatrixXd Covariance;
    std::map<std::string, std::tuple<int, Eigen::Vector2d, Sophus::SO3d> > state_history_;
    PoseState()
    {
        Position.setZero();
        Velocity.setZero();
        Acceleration.setZero();
        Orientation.setQuaternion(Eigen::Quaterniond::Identity());
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
        Covariance = Eigen::MatrixXd::Zero(ndim(), ndim());
        Covariance.setIdentity();
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
        for(auto &keyval : state_history_) {
            std::string key;
            int idx;
            Eigen::Vector2d pos;
            Sophus::SO3d orient;
            key = keyval.first;
            std::tie(idx, pos, orient) = keyval.second;
            pos += offset.segment<2>(idx);
            rot = Sophus::SO3d::exp(offset.segment<3>(idx+2));
            orient = rot*orient;
            out.state_history_[key] = std::make_tuple(idx, pos, rot);
        }
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

        for(auto const &keyval : state_history_) {
            std::string key;
            int idx, otheridx;
            Eigen::Vector2d pos, otherpos;
            Sophus::SO3d orient, otherorient;
            key = keyval.first;
            std::tie(idx, pos, orient) = keyval.second;
            std::tie(otheridx, otherpos, otherorient) = other.state_history_.at(key);
            out.segment<2>(idx) = pos - otherpos;
            out.segment<3>(idx+2) = Sophus::SO3d::log(orient*otherorient.inverse());
        }

        return out;
    };
    struct PoseState
    advance(double dt,
            Eigen::VectorXd control,
            const Eigen::VectorXd &Chinu) const
    {
        struct PoseState out;
        Eigen::Vector3d deltapos;
        deltapos.segment<2>(0) = (dt*Velocity + dt*dt*Acceleration/2.);
        deltapos(2) = 0;
        out.Velocity = Velocity + dt*Acceleration + Chinu.segment<2>(2);
        out.Acceleration = Acceleration + Chinu.segment<2>(4);
        Eigen::Vector3d delta_orientation = Omega*dt + Chinu.segment<3>(6);
        out.Position = Position + (Orientation*deltapos).segment<2>(0) + Chinu.segment<2>(0);
        out.Orientation = Orientation*Sophus::SO3d::exp(delta_orientation);
        out.Omega = Omega + Chinu.segment<3>(9);
        out.GyroBias = GyroBias + Chinu.segment<3>(12);
        out.AccelBias = AccelBias + Chinu.segment<3>(15);
        for(auto &keyval : state_history_) {
            std::string key;
            int idx;
            Eigen::Vector2d pos;
            Sophus::SO3d orient;
            key = keyval.first;
            std::tie(idx, pos, orient) = keyval.second;
            pos += Chinu.segment<2>(idx);
            delta_orientation = Chinu.segment<3>(idx+2);
            orient = orient*Sophus::SO3d::exp(delta_orientation);
            out.state_history_[key] = std::make_tuple(idx, pos, orient);
        }
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
        std::map<std::string, std::tuple<int, Eigen::Vector2d, RotationAverage>> history_average;
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
            for(auto &keyval : pt.state_history_) {
                std::string key;
                int idx;
                Eigen::Vector2d pos;
                Sophus::SO3d orient;
                key = keyval.first;
                std::tie(idx, pos, orient) = keyval.second;
                auto current = history_average.find(key);
                if( current == history_average.end() )
                {
                    current->second = std::make_tuple(idx, w*pos, RotationAverage());
                    std::get<2>(current->second)(w, orient);
                    history_average[current->first] = current->second;
                }
                else
                {
                    std::get<1>(current->second) += w*pos;
                    std::get<2>(current->second)(w, orient);
                }
            }
        }
        double error = ra.geodesic(&Orientation);
        if(error<0)
        {
            Orientation = Chistate.front().Orientation;
        }
        for(auto &keyval : history_average) {
            std::string key;
            int idx;
            Eigen::Vector2d pos;
            RotationAverage ra;
            Sophus::SO3d orient;
            key = keyval.first;
            std::tie(idx, pos, ra) = keyval.second;
            error = ra.geodesic(&orient);
            if(error<0)
            {
                orient = Chistate.front().Orientation;
            }
            state_history_[key] = std::make_tuple(idx, pos, orient);
        }
     };

    void augment(std::string key)
    {
        int index=Covariance.rows();
        state_history_[key] = std::make_tuple(index, Position, Orientation);
        Covariance.conservativeResize(index+5, index+5);
        Covariance.block<5, 5>(index, index) = Covariance.block<5, 5>(0, 0);
        Covariance.block<5, 5>(index+5, index) = Eigen::Matrix<double, 5, 5>::Zero();
        Covariance.block<5, 5>(index, index+5) = Eigen::Matrix<double, 5, 5>::Zero();
    }

    void getAugmentation(std::string key, Eigen::Vector2d& oldPosition, Sophus::SO3d& oldOrientation, Eigen::MatrixXd oldCov)
    {
        int index;
        std::tie(index, oldPosition, oldOrientation) = state_history_[key];
        state_history_.erase(key);
        slice(Covariance, index, 5, oldCov);
    }

    void slice(Eigen::MatrixXd& M, int slicestart, int slicesize, Eigen::MatrixXd& piece)
    {
        int newsize=M.rows() - slicesize;

        // Cut out the piece I want along the diagonal
        piece.resize(slicesize, slicesize);
        piece = M.block(slicestart, slicestart, slicesize, slicesize);

        // Move any remaining values left from the right edge
        M.block(0, slicestart, M.rows(), newsize-slicestart) = M.block(0, slicestart+slicesize, M.rows(), newsize-slicestart);
        // Move any remaining values up from the bottom edge
        M.block(slicestart, 0, newsize-slicestart, newsize) = M.block(slicestart+slicesize, 0, newsize-slicestart, newsize);
        // Cut off the unneeded values
        M.conservativeResize(newsize, newsize);
    }


    ssize_t ndim(void) const {return 3*2 + 4*3 + 5*state_history_.size();};
    static const double littleg;
    static const Eigen::Vector3d gravity;

    friend std::ostream& operator<<(std::ostream &, const PoseState &);
};

struct WheelOdometryMeasurement
{
    double delta_yaw;
    double yaw_rate;
    Eigen::Vector2d delta_pos;
    Eigen::Vector2d velocity;
    const std::string key_="WheelOdometryMeasurement";

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
    measure(struct PoseState& st, const Eigen::VectorXd& noise) const
    {
        struct WheelOdometryMeasurement m;
        Eigen::Vector2d lastPosition;
        Sophus::SO3d lastOrientation;
        Eigen::MatrixXd lastCovariance;
        st.getAugmentation(key_, lastPosition, lastOrientation, lastCovariance);
        Sophus::SO3d::Tangent delta_orientation;
        delta_orientation = Sophus::SO3d::log(st.Orientation*lastOrientation.inverse());
        m.delta_yaw = delta_orientation(2) + noise(0);
        m.yaw_rate = noise(1);
        m.delta_pos = st.Position - lastPosition + noise.segment<2>(2);
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
        m.acceleration = st.Orientation.inverse() * st.gravity + st.AccelBias;
        m.acceleration.segment<2>(0) += st.Acceleration;
        m.acceleration += noise.segment<3>(0);
        m.omega = st.Omega + st.GyroBias + noise.segment<3>(3);
        return m;
    };

    ssize_t ndim(void) const { return 6; };
    friend std::ostream& operator<<(std::ostream &, const IMUOrientationMeasurement &);
};

struct YawMeasurement
{
    double yaw;
    Eigen::Quaterniond qyaw;

    void setyaw(double y)
    {
        yaw = y;
        qyaw = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
    }

    void setyaw(const Eigen::Quaterniond& q)
    {
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(q);
        yaw = m.eulerAngles(0, 1, 2)[2];
    }

    YawMeasurement()
    {
        setyaw(0);
    }

    Eigen::VectorXd
    boxminus(const struct YawMeasurement other) const
    {
        Eigen::VectorXd dy;
        dy.resize(1);
        dy[0] = yaw - other.yaw;
        return dy;
    }

    void mean(const std::vector<double>& weights,
              const std::vector<struct YawMeasurement>& Chimeas)
    {
        double y=0;
        for(const auto &&t: zip_range(weights, Chimeas))
        {
            double w = t.get<0>();
            struct YawMeasurement m = t.get<1>();
            y += w*m.yaw;
        }
        setyaw(y);
    }

    struct YawMeasurement
    measure(const struct PoseState& st, const Eigen::VectorXd& noise) const
    {
        struct YawMeasurement m;
        m.setyaw(st.Orientation.unit_quaternion());
        m.setyaw(yaw+noise[0]);
        return m;
    }

   ssize_t ndim() const { return 1; };
   friend std::ostream& operator<<(std::ostream &, const YawMeasurement &);
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
