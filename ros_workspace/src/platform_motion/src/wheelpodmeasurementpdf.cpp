#include "odometry/wheelpodmeasurementpdf.h"
#include <iostream>

#define MEASMODEL_NUMCONDARGUMENTS_THREEWHEEL 2
#define MEASMODEL_DIMENSION_THREEWHEEL        3

namespace platform_motion
{

WheelPodMeasurementPdf::WheelPodMeasurementPdf(const BFL::Gaussian& measNoise, const MatrixWrapper::Matrix &pod_positions)
    : BFL::AnalyticConditionalGaussianAdditiveNoise(measNoise,MEASMODEL_NUMCONDARGUMENTS_THREEWHEEL),
    pod_positions(pod_positions)
{
}

WheelPodMeasurementPdf::~WheelPodMeasurementPdf(){}

MatrixWrapper::Matrix
WheelPodMeasurementPdf::computePodDirections(void) const
{
    MatrixWrapper::ColumnVector control = ConditionalArgumentGet(1);
    MatrixWrapper::Matrix pod_direction(3,3);
    pod_direction(1,1) = cos(control(1));
    pod_direction(2,1) = sin(control(1));
    pod_direction(3,1) = 0;
    pod_direction(1,2) = cos(control(2));
    pod_direction(2,2) = sin(control(2));
    pod_direction(3,2) = 0;
    pod_direction(1,3) = cos(control(3));
    pod_direction(2,3) = sin(control(3));
    pod_direction(3,3) = 0;
    return pod_direction;
}

MatrixWrapper::Matrix
WheelPodMeasurementPdf::dfGet(unsigned int i) const
{
    MatrixWrapper::Matrix pod_direction = computePodDirections();
    MatrixWrapper::Matrix df(3,6);

    if(i==0)
    {
        //[[ dv0/dx     dv0/dy     dv0/dtheta dv0/dvx    dv0/dvy    dv0/domega]
        // [ dv1/dx     dv1/dy     dv1/dtheta dv1/dvx    dv1/dvy    dv1/domega]
        // [ dv2/dx     dv2/dy     dv2/dtheta dv2/dvx    dv2/dvy    dv2/domega]]

        df(1,1) = 0; df(1,2) = 0; df(1,3) = 0;
        df(1,4) = pod_direction(1,1);
        df(1,5) = pod_direction(2,1);
        df(1,6) = -1.0*pod_positions(2,1)*pod_direction(1,1)
                  +    pod_positions(1,1)*pod_direction(2,1);

        df(2,1) = 0; df(2,2) = 0; df(2,3) = 0;
        df(2,4) = pod_direction(1,2);
        df(2,5) = pod_direction(2,2);
        df(2,6) = -1*pod_positions(2,2)*pod_direction(1,2)
                  +  pod_positions(1,2)*pod_direction(2,2);

        df(3,1) = 0; df(3,2) = 0; df(3,3) = 0;
        df(3,4) = pod_direction(1,3);
        df(3,5) = pod_direction(2,3);
        df(3,6) = -1*pod_positions(2,3)*pod_direction(1,3)
                  +  pod_positions(1,3)*pod_direction(2,3);
    }
    else
    {
        std::cerr << "derivative not supported";
    }
    std::cerr << "df: " << df << std::endl;
    return df;
}

MatrixWrapper::ColumnVector
WheelPodMeasurementPdf::ExpectedValueGet(void) const
{
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector z(3);

    MatrixWrapper::ColumnVector vel(3);
    vel(1) = state(4);
    vel(2) = state(5);
    vel(3) = 0;
    std::cerr << "state: " << state << std::endl;
    std::cerr << "vel: " << vel << std::endl;

    MatrixWrapper::Matrix omegaCross(3,3);
    omegaCross = 0;
    omegaCross(1, 2) = -state(6);
    omegaCross(2, 1) =  state(6);

    MatrixWrapper::Matrix pod_direction = computePodDirections();
    std::cerr << "dir: " << pod_direction << std::endl;

    z(1) = (vel + omegaCross*pod_positions.columnCopy(1)).transpose()*pod_direction.columnCopy(1);
    z(2) = (vel + omegaCross*pod_positions.columnCopy(2)).transpose()*pod_direction.columnCopy(2);
    z(3) = (vel + omegaCross*pod_positions.columnCopy(3)).transpose()*pod_direction.columnCopy(3);

    std::cerr << "z: " << z << std::endl;
    return z;
}

}
