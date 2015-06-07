#include "odometry/odometrywithcurvature.h"

namespace platform_odometry
{

MatrixWrapper::Matrix OdometryWithCurvature::dfGet(unsigned int i) const
{
    MatrixWrapper::ColumnVector x = ConditionalArgumentGet(0);
    MatrixWrapper::Matrix jac(5,5);
    jac = 0.0;
    if(i==0)
    {
        jac(1,1) = 1.0;
        jac(2,2) = 1.0;
        jac(3,3) = 1.0;
        double dl = hypot(x(1), x(2));
        if(dl>1e-5) {
            jac(4,1) = x(1)/dl; jac(4,2) = x(2)/dl;
        }
        jac(4, 3) = 1.0, jac(4,4) = 1.0; jac(4,5) = dl;
        jac(5,5) = 1.0;
    }
    return jac;
}

MatrixWrapper::ColumnVector OdometryWithCurvature::ExpectedValueGet() const
{
    MatrixWrapper::ColumnVector x = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector xprime(5);

    xprime(1) = x(1);
    xprime(2) = x(2);
    xprime(3) = x(3);
    double dl = hypot(x(1), x(2));
    xprime(4) = x(4) + x(3) + x(5)*dl;
    xprime(5) = x(5);

    return xprime + AdditiveNoiseMuGet();
}

}
