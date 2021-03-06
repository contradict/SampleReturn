#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <boost/math/special_functions/sinc.hpp>
#include <pose_ukf/zip_range.hpp>

namespace UKF {

template <typename S>
class ScaledUKF {

    double kappa_, alpha_, beta_;

    S state_;

    void
    generateSigmaPoints(const S& mu,
                        const Eigen::MatrixXd& cov,
                        const std::vector<Eigen::MatrixXd>& covs,
                        std::vector<S>& Chi,
                        std::vector<Eigen::VectorXd>& nu,
                        std::vector<double>& weights,
                        std::vector<double>& cweights
                        ) const
    {
        int L, Lx, Lnu;
        Lx = mu.ndim();
        Lnu = std::accumulate(covs.begin(), covs.end(),
                                    0,
                                    [](int n, const Eigen::MatrixXd& m)
                                    {
                                        return n+m.rows();
                                    });

        L = Lx + Lnu;
        double lambda = alpha_*alpha_*((double)L + kappa_) - (double)L;
        double w0 = lambda/((double)L + lambda);
        double cw0 = w0 + 1.0 - alpha_*alpha_ + beta_;
        double wi = 1.0/((double)L + lambda)/2.0;

        Chi.clear();
        Chi.push_back(mu);
        nu.clear();
        Eigen::VectorXd nuzero(Lnu);
        nuzero.setZero();
        nu.push_back(nuzero);
        weights.clear();
        weights.push_back(w0);
        cweights.clear();
        cweights.push_back(cw0);

        Eigen::MatrixXd P(L, L);
        P.setZero();
        P.block(0, 0, Lx, Lx) = cov;
        int d=Lx;
        for(const Eigen::MatrixXd &m : covs)
        {
            P.block(d, d, m.rows(), m.cols()) = m;
            d += m.rows();
        }

        if((P-P.transpose()).cwiseAbs().maxCoeff()>1e-10)
        {
            ROS_ERROR_STREAM("Augmented covariance not symmetric:\n" << P);
            ROS_ERROR_STREAM("P-P.T:\n" << (P-P.transpose()));
        }
        if(P.determinant() <= 0)
        {
            ROS_ERROR_STREAM("Augmented covariance not positive-definite: " << P.determinant() << "\n" << P);
        }
        Eigen::LLT<Eigen::MatrixXd> llt(((double)L+lambda)*P);
        Eigen::MatrixXd Psqrt = llt.matrixL();
        for(int i=0;i<L;i++)
        {
            Eigen::VectorXd p=Psqrt.col(i);
            Chi.push_back(mu.boxplus(p.segment(0, Lx)));
            nu.push_back(p.segment(Lx, Lnu));
            weights.push_back(wi);
            cweights.push_back(wi);

            Chi.push_back(mu.boxplus(-1.0*p.segment(0, Lx)));
            nu.push_back(-1.0*p.segment(Lx, Lnu));
            weights.push_back(wi);
            cweights.push_back(wi);
        }
    }


    public:
        ScaledUKF(double alpha=1e-1, double beta=2.0, double kappa=0.0) :
            alpha_(alpha),
            beta_(beta),
            kappa_(kappa)
        {
        };

        void
        reset(const S& st)
        {
            state_ = st;
        }

        void
        predict(double dt, Eigen::MatrixXd process_noise)
        {
            predict(dt, Eigen::VectorXd::Zero(process_noise.rows()), process_noise);
        }

        void
        predict(double dt, Eigen::VectorXd control, Eigen::MatrixXd process_noise)
        {
            std::vector<Eigen::MatrixXd> covs;
            covs.push_back(process_noise);
            std::vector<S> Chi;
            std::vector<Eigen::VectorXd> nu;
            std::vector<double> weights, cweights;
            generateSigmaPoints(state_, state_.Covariance, covs,
                    Chi, nu, weights, cweights);

            std::vector<S> Chiminus;
            for(auto && t: zip_range(Chi, nu))
            {
                S chi = boost::get<0>(t);
                Eigen::VectorXd nu = boost::get<1>(t);
                S st = chi.advance(dt, control, nu);
                Chiminus.push_back(st);
            }
            S xhatminus;
            xhatminus.mean(weights, Chiminus);

            Eigen::MatrixXd Pminus(state_.ndim(), state_.ndim());
            Pminus.setZero();
            for(auto && t: zip_range(cweights, Chiminus))
            {
                double w = boost::get<0>(t);
                S pt = boost::get<1>(t);
                Eigen::VectorXd diff = pt.boxminus(xhatminus);
                Pminus += w * diff * diff.transpose();
            }

            state_ = xhatminus;
            state_.Covariance = Pminus;
        }

        template <typename M>
        void
        differentialcorrect(const M& measured,
                const std::vector<Eigen::MatrixXd>& measurement_covs
               )
        {
            correct(measured, measurement_covs);
            state_.augment(measured.key_);
        }

        template <typename M>
        void
        correct(const M& measured,
                const std::vector<Eigen::MatrixXd>& measurement_covs
               )
        {
            std::vector<Eigen::MatrixXd> covs;
            covs.insert(covs.end(),
                    measurement_covs.begin(), measurement_covs.end());
            std::vector<S> Chi;
            std::vector<Eigen::VectorXd> nu;
            std::vector<double> weights, cweights;
            generateSigmaPoints(state_, state_.Covariance, covs,
                    Chi, nu, weights, cweights);


            std::vector<M> Yminus;
            for(const auto && t:zip_range(Chi, nu))
            {
                S st = boost::get<0>(t);
                Eigen::VectorXd nu = boost::get<1>(t);
                Yminus.push_back(measured.measure(st, nu));
            }

            M yhatminus;
            yhatminus.mean(weights, Yminus);

            int Lx = state_.ndim();
            Eigen::MatrixXd Pxy(Lx, yhatminus.ndim());
            Pxy.setZero();
            Eigen::MatrixXd Pyminus(measured.ndim(), measured.ndim());
            Pyminus.setZero();

            S chi0 = Chi[0];
            for(const auto && t: zip_range(cweights, Chi, Yminus))
            {
                double w = boost::get<0>(t);
                Eigen::VectorXd xdiff = boost::get<1>(t).boxminus(chi0);
                Eigen::VectorXd ydiff = boost::get<2>(t).boxminus(yhatminus);
                Pyminus += w * ydiff * ydiff.transpose();
                Pxy += w * xdiff * ydiff.transpose();
            }

            Eigen::MatrixXd K(yhatminus.ndim(), Lx);
            K = Pxy*Pyminus.inverse();
            Eigen::VectorXd innovation = (measured.boxminus(yhatminus));
            Eigen::VectorXd update = K*innovation;
            state_ = state_.boxplus(update);
            state_.Covariance -= K*Pyminus*K.transpose();
            for(int i=0;i<state_.Covariance.size();i++)
            {
                if(isnan(*(state_.Covariance.data()+i)) ||
                   fabs(*(state_.Covariance.data()+i))>10.0f)
                {
                    ROS_ERROR_STREAM("Silly correct covariance\n" << state_.Covariance);
                    break;
                }
            }
        }

        const S& state(void) const {return state_;};

        int ndim(void) const { return state_.ndim(); };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};
