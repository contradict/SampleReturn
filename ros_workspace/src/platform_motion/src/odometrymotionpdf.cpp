#include "odometry/odometrymotionpdf.h"
#include <iostream>

#define NUMCONDARGUMENTS 1

namespace platform_motion
{

OdometryMotionPdf::OdometryMotionPdf(const BFL::Gaussian& sysNoise)
    : BFL::AnalyticConditionalGaussianAdditiveNoise(sysNoise,NUMCONDARGUMENTS)
{
}

OdometryMotionPdf::~OdometryMotionPdf(){}

MatrixWrapper::Matrix
OdometryMotionPdf::dfGet(unsigned int i) const
{
    MatrixWrapper::Matrix df(6,6);
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);

    if(i==0)
    {
        df = 0;
        //[[ dx/dx      dx/dy      dx/dtheta      dx/dvx      dx/dvy      dx/domega      ]
        // [ dy/dx      dy/dy      dy/dtheta      dy/dvx      dy/dvy      dy/domega      ]
        // [ dtheta/dx  dtheta/dy  dtheta/dtheta  dtheta/dvx  dtheta/dvy  dtheta/domega  ]
        // [ dvx/dx     dvx/dy     dvx/dtheta     dvx/dvx     dvx/dvy     dvx/domega     ]
        // [ dvy/dx     dvy/dy     dvy/dtheta     dvy/dvx     dvy/dvy     dvy/domega     ]
        // [ domega/dx  domega/dy  domega/dtheta  domega/dvx  domega/dvy  domega/domega ]]
        df(1,1) = 1.0;
        df(1,2) = 0.0;
        df(1,3) =-state(4)*sin(state(3));
        df(1,4) = cos(state(3));
        df(1,5) = 0.0;
        df(1,6) = 0.0;

        df(2,1) = 0.0;
        df(2,2) = 1.0;
        df(2,3) = state(4)*cos(state(3));
        df(2,4) = sin(state(3));
        df(2,5) = 1.0;
        df(2,6) = 0.0;

        df(3,1) = 0.0;
        df(3,2) = 0.0;
        df(3,3) = 1.0;
        df(3,4) = 0.0;
        df(3,5) = 0.0;
        df(3,6) = 1.0;

        df(4,4) = 1.0;

        df(5,5) = 1.0;

        df(6,6) = 1.0;
    }
    else
    {
        std::cerr << "derivative not supported";
    }
    std::cerr << "df: " << df << std::endl;
    return df;
}

MatrixWrapper::ColumnVector
OdometryMotionPdf::ExpectedValueGet(void) const
{
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);

    state(1) += state(4)*cos(state(3));
    state(2) += state(4)*sin(state(3)) + state(5);
    state(3) += state(6);

    return state + AdditiveNoiseMuGet();
}

}
