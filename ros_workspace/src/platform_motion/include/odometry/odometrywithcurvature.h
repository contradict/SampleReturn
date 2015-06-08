#ifndef __ODOMETRY_WITH_CURVATURE_H_
#define __ODOMETRY_WITH_CURVATURE_H_

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace platform_odometry
{

class OdometryWithCurvature : public BFL::AnalyticConditionalGaussianAdditiveNoise
{
    public:
        OdometryWithCurvature(const BFL::Gaussian& gaus) :
            BFL::AnalyticConditionalGaussianAdditiveNoise(gaus) {};
        MatrixWrapper::Matrix dfGet(unsigned int i) const;
        MatrixWrapper::ColumnVector ExpectedValueGet(void) const;
};

}

#endif // __ODOMETRY_WITH_CURVATURE_H_
