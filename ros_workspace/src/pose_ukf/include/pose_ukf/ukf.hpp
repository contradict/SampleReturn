#include <ros/ros.h>
#include <vector>
#include <tuple>
#include <functional>
#include <algorithm>
#include <Eigen/Dense>
#include <boost/math/special_functions/sinc.hpp>
#include <pose_ukf/zip_range.hpp>

namespace UKF {

template <class T, class S>
struct Measurement {
    virtual Eigen::VectorXd boxminus(const T& other) const = 0;
    virtual void mean(const std::vector<double>& weights,
              const std::vector<T>& Chimeas) = 0;

    virtual T measure(const S& state, const Eigen::VectorXd& noise) const = 0;
    virtual ssize_t ndim(void) const = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <class S>
struct State {
    virtual S boxplus(const Eigen::VectorXd& offset) const = 0;
    virtual Eigen::VectorXd boxminus(const S& other) const = 0;
    virtual S advance(double dt, const Eigen::VectorXd& Chinu) const = 0;
    virtual void mean(const std::vector<double>& weights,
                      const std::vector<S>& Chistate) = 0;
    virtual ssize_t ndim(void) const = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename S>
class ScaledUKF {

    double kappa_, alpha_, beta_;

    S state_;
    Eigen::MatrixXd covariance_;

    std::vector<S> Chistate_;
    std::vector<S> Chiminus_;
    std::vector<Eigen::VectorXd> Chinu_;
    int Lx_, Lnu_, L_;
    std::vector<double> weights_;
    std::vector<double> cweights_;
    S xhatminus_;
    Eigen::MatrixXd Pminus_;
    std::vector<Eigen::VectorXd> xdiff_;

    void
    generateSigmaPoints(const std::vector<Eigen::MatrixXd>& covs)
    {
        Lx_ = ndim();
        Lnu_ = std::accumulate(covs.begin(), covs.end(),
                                    0,
                                    [](int n, const Eigen::MatrixXd& m)
                                    {
                                        return n+m.rows();
                                    });

        L_ = Lx_ + Lnu_;
        double lambda = alpha_*alpha_*((double)L_ + kappa_) - (double)L_;
        double w0 = lambda/((double)L_ + lambda);
        double cw0 = w0 + 1.0 - alpha_*alpha_ + beta_;
        double wi = 1.0/((double)L_ + lambda)/2.0;

        Chistate_.clear();
        Chistate_.push_back(state_);
        Chinu_.clear();
        Eigen::VectorXd nuzero(Lnu_);
        nuzero.setZero();
        Chinu_.push_back(nuzero);
        weights_.clear();
        weights_.push_back(w0);
        cweights_.clear();
        cweights_.push_back(cw0);

        Eigen::MatrixXd P(L_, L_);
        P.setZero();
        P.block(0, 0, Lx_, Lx_) = covariance_;
        int d=Lx_;
        for(const Eigen::MatrixXd &m : covs)
        {
            P.block(d, d, m.rows(), m.cols()) = m;
            d += m.rows();
        }

        Eigen::LLT<Eigen::MatrixXd> llt(((double)L_+lambda)*P);
        Eigen::MatrixXd Psqrt = llt.matrixL();
        for(int i=0;i<L_;i++)
        {
            Eigen::VectorXd p=Psqrt.col(i);
            Chistate_.push_back(state_.boxplus(p.segment(0, Lx_)));
            Chinu_.push_back(p.segment(Lx_, Lnu_));
            weights_.push_back(wi);
            cweights_.push_back(wi);

            Chistate_.push_back(state_.boxplus(-1.0*p.segment(0, Lx_)));
            Chinu_.push_back(-1.0*p.segment(Lx_, Lnu_));
            weights_.push_back(wi);
            cweights_.push_back(wi);
        }
    }

    protected:
        int ndim(void) const { return state_.ndim(); };

        virtual Eigen::MatrixXd
        process_noise(double dt) const = 0;

    public:
        ScaledUKF(double alpha=1e-1, double beta=2.0, double kappa=0.0) :
            alpha_(alpha),
            beta_(beta),
            kappa_(kappa)
        {
            covariance_.resize(ndim(), ndim());
            covariance_.setIdentity();
        };

        void
        reset(const S& st, const Eigen::MatrixXd& cov)
        {
            state_ = st;
            covariance_ = cov;
        }

        void
        predict(double dt,
                const std::vector<Eigen::MatrixXd>& measurement_covs,
                bool predict_only=false
                )
        {
            std::vector<Eigen::MatrixXd> covs;
            covs.push_back(process_noise(dt));
            covs.insert(covs.end(),
                    measurement_covs.begin(), measurement_covs.end());
            generateSigmaPoints(covs);

            Chiminus_.clear();
            for(auto && t: zip_range(Chistate_, Chinu_))
            {
                S chi = boost::get<0>(t);
                Eigen::VectorXd nu = boost::get<1>(t);
                S st = chi.advance(dt, nu.segment(0, Lx_));
                Chiminus_.push_back(st);
            }
            xhatminus_.mean(weights_, Chiminus_);

            xdiff_.clear();
            Pminus_.resize(Lx_, Lx_);
            Pminus_.setZero();
            for(auto && t: zip_range(cweights_, Chiminus_))
            {
                double w = boost::get<0>(t);
                S pt = boost::get<1>(t);
                Eigen::VectorXd diff = pt.boxminus(xhatminus_);
                xdiff_.push_back(diff);
                Pminus_ += w * diff * diff.transpose();
            }

            if(predict_only)
            {
                state_ = xhatminus_;
                covariance_ = Pminus_;
            }
        }

        template <typename M>
        void
        correct(const M& measured)
        {
            std::vector<M> Yminus;
            for(const auto && t:zip_range(Chiminus_, Chinu_))
            {
                S st = boost::get<0>(t);
                Eigen::VectorXd nu = boost::get<1>(t);
                Yminus.push_back(measured.measure(st,
                            nu.segment(Lx_, measured.ndim())));
            }

            M yhatminus;
            yhatminus.mean(weights_, Yminus);

            Eigen::MatrixXd Pxy(Lx_, yhatminus.ndim());
            Pxy.setZero();
            Eigen::MatrixXd Pyminus(measured.ndim(), measured.ndim());
            Pyminus.setZero();

            for(const auto && t: zip_range(cweights_, xdiff_, Yminus))
            {
                double w = boost::get<0>(t);
                Eigen::VectorXd xdiff = boost::get<1>(t);
                M m = boost::get<2>(t);
                Eigen::VectorXd ydiff = m.boxminus(yhatminus);
                Pyminus += w * ydiff * ydiff.transpose();
                Pxy += w * xdiff * ydiff.transpose();
            }

            Eigen::MatrixXd K(yhatminus.ndim(), Lx_);
            K = Pxy*Pyminus.inverse();
            state_ = xhatminus_.boxplus(K*(measured.boxminus(yhatminus)));
            covariance_ = Pminus_ - K*Pyminus*K.transpose();
        }

        const S& state(void) {return state_;};
        const Eigen::MatrixXd& covariance(void) {return covariance_;};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};
