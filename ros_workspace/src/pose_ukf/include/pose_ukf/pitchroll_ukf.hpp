#include <pose_ukf/ukf.hpp>
#include <ostream>
#include <sophus/so3.hpp>
#include <pose_ukf/rotation_average.h>

#define USE_YAW false

namespace PitchRollUKF {

struct PitchRollState
{
    Sophus::SO3d Orientation;
    Eigen::Vector3d Omega;
    Eigen::Vector3d GyroBias;
    Eigen::Vector3d AccelBias;
    Eigen::MatrixXd Covariance;
    PitchRollState()
    {
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
        Covariance.resize(ndim(), ndim());
        Covariance.setIdentity();
    };
    struct PitchRollState
    boxplus(const Eigen::VectorXd& offset) const
    {
        struct PitchRollState out;
        Eigen::Vector3d omega;
        omega.setZero();
        omega = offset.segment<3>(0);
        out.Omega = Omega + offset.segment<3>(3);
        out.GyroBias = GyroBias + offset.segment<3>(6);
        Sophus::SO3d rot=Sophus::SO3d::exp(omega);
        out.Orientation = rot*Orientation;
        out.AccelBias = AccelBias + offset.segment<3>(9);
        out.Covariance = Covariance;
        return out;
    };
    Eigen::VectorXd
    boxminus(const struct PitchRollState& other) const
    {
        Eigen::VectorXd out(ndim());

        out.segment<3>(0) = Sophus::SO3d::log(Orientation*other.Orientation.inverse());
        out.segment<3>(3) = Omega - other.Omega;
        out.segment<3>(6) = GyroBias - other.GyroBias;
        out.segment<3>(9) = AccelBias - other.AccelBias;

        return out;
    };
    struct PitchRollState
    advance(double dt,
            const Eigen::VectorXd& control,
            const Eigen::VectorXd &Chinu) const
    {
        (void)control;
        struct PitchRollState out;
        Eigen::Vector3d delta_orientation = Omega*dt + Chinu.segment<3>(0);
        out.Orientation = Orientation*Sophus::SO3d::exp(delta_orientation);
        out.Omega = Omega + Chinu.segment<3>(3);
        out.GyroBias = GyroBias + Chinu.segment<3>(6);
        out.AccelBias = AccelBias + Chinu.segment<3>(9);
        out.Covariance = Covariance;
        return out;
    };
    void mean(const std::vector<double> &weights,
              const std::vector<struct PitchRollState> &Chistate)
    {
        Omega.setZero();
        GyroBias.setZero();
        AccelBias.setZero();
        RotationAverage ra;
        for(auto && t: zip_range(weights, Chistate))
        {
            double w=t.get<0>();
            const struct PitchRollState& pt = t.get<1>();
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
        Covariance = Chistate[0].Covariance;
    };
    ssize_t ndim(void) const {return 4*3;};

    friend std::ostream& operator<<(std::ostream &, const PitchRollState &);
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
    measure(const struct PitchRollState& st, const Eigen::VectorXd& noise) const
    {
        struct IMUOrientationMeasurement m;
        m.acceleration = -1.0*(st.Orientation.inverse() * Eigen::Vector3d::UnitZ()) + st.AccelBias + noise.segment<3>(0);
        m.omega = st.Omega + st.GyroBias + noise.segment<3>(3);
        return m;
    };

    ssize_t ndim(void) const { return 6; };
    friend std::ostream& operator<<(std::ostream &, const IMUOrientationMeasurement &);
};

struct YawMeasurement
{
    Sophus::SO3d yaw;

    YawMeasurement()
    {
        yaw.setQuaternion(Eigen::Quaterniond::Identity());
    }

    void project(void)
    {
        Eigen::Vector3d t = Sophus::SO3d::log(yaw);
        t.segment<2>(0).setZero();
        yaw = Sophus::SO3d::exp(t);
    }

    Eigen::VectorXd
    boxminus(const struct YawMeasurement other) const
    {
        Eigen::VectorXd dy(ndim());
        Eigen::Vector3d dq = Sophus::SO3d::log(yaw*other.yaw.inverse());
        dy[0] = dq.z();
        return dy;
    }

    void mean(const std::vector<double>& weights,
              const std::vector<struct YawMeasurement>& Chimeas)
    {
        RotationAverage ra;
        for(const auto &&t: zip_range(weights, Chimeas))
        {
            double w = t.get<0>();
            struct YawMeasurement m = t.get<1>();
            ra(w, m.yaw);
        }
        double error = ra.geodesic(&yaw);
        if(error<0)
        {
            yaw = Chimeas.front().yaw;
        }
        project();
    }

    struct YawMeasurement
    measure(const struct PitchRollState& st, const Eigen::VectorXd& noise) const
    {
        struct YawMeasurement m;
        m.yaw = st.Orientation;
        Eigen::Vector3d dq(0,0,noise[0]);
        m.yaw = m.yaw*Sophus::SO3d::exp(dq);
        m.project();
        return m;
    }

    ssize_t ndim() const { return 1; };
    friend std::ostream& operator<<(std::ostream &, const YawMeasurement &);
};

class PitchRollUKF : public UKF::ScaledUKF<struct PitchRollState>
{

    public:

    PitchRollUKF(double alpha=1.0, double beta=2.0, double kappa=0.0) :
        ::UKF::ScaledUKF<struct PitchRollState>(alpha, beta, kappa)
    {
    };
};

}
