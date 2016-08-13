#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <pose_ukf/rotation_average.h>
#include <pose_ukf/zip_range.hpp>

namespace PoseUKF {

template<typename StateT>
struct YawMeasurement;

template<typename StateT>
std::ostream &
operator<<(std::ostream &out, const YawMeasurement<StateT>& m);


template<typename StateT>
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
            double w = boost::get<0>(t);
            struct YawMeasurement m = boost::get<1>(t);
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
    measure(const StateT& st, const Eigen::VectorXd& noise) const
    {
        struct YawMeasurement m;
        m.yaw = st.Orientation;
        Eigen::Vector3d dq(0,0,noise[0]);
        m.yaw = m.yaw*Sophus::SO3d::exp(dq);
        m.project();
        return m;
    }

    ssize_t ndim() const { return 1; };
    friend std::ostream& operator<< <>(std::ostream &, const YawMeasurement &);
};

template<typename StateT>
std::ostream &
operator<<(std::ostream &out, const YawMeasurement<StateT>& m)
{
    out << "yaw: " << 2*acos(m.yaw.unit_quaternion().coeffs().z()) << std::endl;
    return out;
}

}
