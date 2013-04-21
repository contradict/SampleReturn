#ifndef __NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_THREE_WHEEL_H__
#define __NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_THREE_WHEEL_H__

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace platform_odometry
{

class NonLinearAnalyticConditionalGaussianThreeWheel : public BFL::AnalyticConditionalGaussianAdditiveNoise
{
    public:
        NonLinearAnalyticConditionalGaussianThreeWheel( const BFL::Gaussian& additiveNoise );

        virtual ~NonLinearAnalyticConditionalGaussianThreeWheel();

        virtual MatrixWrapper::ColumnVector     ExpectedValueGet() const;
        virtual MatrixWrapper::Matrix           dfGet(unsigned int i) const;
};

}

#endif // __NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_THREE_WHEEL_H__
