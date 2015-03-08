#include <pose_ukf/ukf.hpp>
#include <algorithm>
#include <random>


namespace PoseUKF {

struct TestState : public UKF::State<struct TestState> {
    double x, v;

    TestState() :
        x(0.0),
        v(0.0)
    {};

    struct TestState
    boxplus(const Eigen::VectorXd& offset) const
    {
        struct TestState st;
        st.x = x + offset(0);
        st.v = v + offset(1);
        return st;
    };

    Eigen::VectorXd
    boxminus(const struct TestState& other) const
    {
        Eigen::VectorXd offset(ndim());
        offset(0) = x - other.x;
        offset(1) = v - other.v;
        return offset;
    }

    struct TestState
    advance(double dt, const Eigen::VectorXd& nu) const
    {
        struct TestState st;
        st.x = x + v*dt + nu(0);
        st.v = v + nu(1);
        return st;
    };

    void
    mean(const std::vector<double>& weights,
            const std::vector<struct TestState>& states)
    {
        x=0;
        v=0;
        for( auto&& t : zip_range(weights, states))
        {
            double w=boost::get<0>(t);
            struct TestState st=boost::get<1>(t);
            x += w*st.x;
            v += w*st.v;
        }
    };

    ssize_t ndim(void) const {return 2;};
};

struct PositionMeasurement : UKF::Measurement<struct PositionMeasurement, struct TestState>
{
    double x;

    PositionMeasurement() :
    x(0.0)
    {};

    Eigen::VectorXd
    boxminus(const struct PositionMeasurement& other) const
    {
        Eigen::VectorXd offset(ndim());
        offset(0) = x-other.x;
        return offset;
    };

    void
    mean(const std::vector<double>& weights,
        const std::vector<PositionMeasurement>& measurements)
    {
        x=0;
        for( auto&& t : zip_range(weights, measurements))
        {
            double w=boost::get<0>(t);
            struct PositionMeasurement m=boost::get<1>(t);
            x += w*m.x;
        }
    }

    struct PositionMeasurement
    measure(const struct TestState& st,
            const Eigen::VectorXd& nu) const
    {
        struct PositionMeasurement m;
        m.x = st.x + nu(0);
        return m;
    };

    ssize_t ndim(void) const {return 1;};
};

class TestUKF : public UKF::ScaledUKF<struct TestState>
{
    Eigen::MatrixXd
    process_noise(double dt) const
    {
        Eigen::MatrixXd noise(2,2);
        noise << dt*dt*dt*dt/4.0, dt*dt*dt/2.0,
                      dt*dt*dt/2.0, dt*dt;
        noise *= sigma*sigma;
        return noise;
    };
public:
    TestUKF(double al, double be, double ka) :
        ScaledUKF(al, be, ka)
    {};
    double sigma;
};

}

template
void
UKF::ScaledUKF<struct PoseUKF::TestState>::correct(const struct PoseUKF::PositionMeasurement& measured,
                                                   const std::vector<Eigen::MatrixXd>& measurement_covs
                                                  );

namespace PoseUKF {

void
runTest(void)
{
    double t=0;
    double dt=0.01;
    int N=400;
    std::vector<double> positions;
    std::vector<double> velocities;
    positions.resize(N);
    velocities.resize(N);
    std::generate_n(positions.begin(),
            N,
            [&t, &dt](void)
            {
               double x=sin(t*2*M_PI);
               t += dt;
               return x;
            });
    t = 0.0;
    std::generate_n(velocities.begin(),
            N,
            [&t, &dt](void)
            {
               double x=2*M_PI*cos(t*2*M_PI);
               t += dt;
               return x;
            });
    std::vector<double> noisy_positions;
    noisy_positions.resize(N);
    std::mt19937 generator;
    std::normal_distribution<double> normal(0.0, 0.1);
    std::transform(positions.begin(),
            positions.end(),
            noisy_positions.begin(),
            [&normal, &generator](double x)
            {
               return x+normal(generator);
            });

    TestUKF ukf(1e-3, 2.0, 0.0);
    // acceleration noise
    ukf.sigma = (2*M_PI)*(2*M_PI)*10;
    TestState st;
    st.v=0.0;
    Eigen::MatrixXd cov(2,2);
    cov.setIdentity();
    ukf.reset(st, cov);

    Eigen::MatrixXd measurement_noise(1,1);
    measurement_noise(0,0) = 0.01;
    std::vector<Eigen::MatrixXd> mcovs;
    mcovs.push_back(measurement_noise);

    t = 0.0;
    std::cout << t << ", " <<
                 positions[0] << ", " <<
                 velocities[0] << ", " <<
                 ukf.state().x << ", " <<
                 sqrt(ukf.covariance()(0,0)) << ", " <<
                 ukf.state().v << ", " <<
                 sqrt(ukf.covariance()(1,1)) << ", " <<
                 0.0 << std::endl;
    int correct_every=1;
    for(int i=0;i<N;i++)
    {
        ukf.predict(dt);
        t += dt;
        PositionMeasurement m;
        m.x = noisy_positions[i];
        if(i%correct_every==0)
            ukf.correct(m, mcovs);
        std::cout << t << ", " <<
                     positions[i] << ", " <<
                     velocities[i] << ", " <<
                     ukf.state().x << ", " <<
                     sqrt(ukf.covariance()(0,0)) << ", " <<
                     ukf.state().v << ", " <<
                     sqrt(ukf.covariance()(1,1)) << ", " <<
                     m.x << ", " <<
                     sqrt(measurement_noise(0,0)) <<
                     std::endl;
    }
}

}

int main(int argc, char **argv)
{
    PoseUKF::runTest();
}
