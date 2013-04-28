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
    MatrixWrapper::Matrix df(6,3);

    if(i==0)
    {
        //[[ dv0/dvx         dv0/dvy         dv0/domega]
        // [ dv0_perp/dvx    dv0_perp/dvy    dv0_perp/domega]
        // [ dv1/dvx         dv1/dvy         dv1/domega]
        // [ dv1_perp/dvx    dv1_perp/dvy    dv1_perp/domega]
        // [ dv2/dvx         dv2/dvy         dv2/domega]]
        // [ dv2_perp/dvx    dv2_perp/dvy    dv2_perp/domega]]

        df(1,1) =      pod_direction(1,1);
        df(1,2) =      pod_direction(2,1);
        df(1,3) = -1.0*pod_positions(2,1)*pod_direction(1,1)
                  +    pod_positions(1,1)*pod_direction(2,1);

        df(2,1) = -1.0*pod_direction(2,1); 
        df(2,2) =      pod_direction(1,1); 
        df(2,3) =      pod_positions(2,1)*pod_direction(2,1)
                  +    pod_positions(1,1)*pod_direction(1,1);

        df(3,1) =      pod_direction(1,2);
        df(3,2) =      pod_direction(2,2);
        df(3,3) = -1.0*pod_positions(2,2)*pod_direction(1,2)
                  +    pod_positions(1,2)*pod_direction(2,2);

        df(4,1) = -1.0*pod_direction(2,2); 
        df(4,2) =      pod_direction(1,2); 
        df(4,3) =      pod_positions(2,2)*pod_direction(2,2)
                  +    pod_positions(1,2)*pod_direction(1,2);

        df(5,1) = pod_direction(1,3);
        df(5,2) = pod_direction(2,3);
        df(5,3) = -1*pod_positions(2,3)*pod_direction(1,3)
                  +  pod_positions(1,3)*pod_direction(2,3);

        df(6,1) = -1.0*pod_direction(2,3); 
        df(6,2) =      pod_direction(1,3); 
        df(6,3) =      pod_positions(2,3)*pod_direction(2,3)
                  +    pod_positions(1,3)*pod_direction(1,3);
    }
    else
    {
        std::cerr << "derivative not supported";
    }
    return df;
}

MatrixWrapper::ColumnVector
WheelPodMeasurementPdf::ExpectedValueGet(void) const
{
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector z(6);

    MatrixWrapper::ColumnVector vel(3);
    vel(1) = state(1);
    vel(2) = state(2);
    vel(3) = 0;

    MatrixWrapper::Matrix zCross(3,3);
    zCross = 0;
    zCross(1, 2) = -1;
    zCross(2, 1) =  1;

    MatrixWrapper::Matrix pod_direction = computePodDirections();
    MatrixWrapper::Matrix pod_direction_perp = zCross*pod_direction;

    MatrixWrapper::Matrix pod_circular_vel = zCross*pod_positions*state(3);

    MatrixWrapper::ColumnVector podvel;
    podvel = (vel + pod_circular_vel.columnCopy(1));
    z(1) = podvel.transpose()*pod_direction.columnCopy(1);
    z(2) = podvel.transpose()*pod_direction_perp.columnCopy(1);
    podvel = (vel + pod_circular_vel.columnCopy(2));
    z(3) = podvel.transpose()*pod_direction.columnCopy(2);
    z(4) = podvel.transpose()*pod_direction_perp.columnCopy(2);
    podvel = (vel + pod_circular_vel.columnCopy(3));
    z(5) = podvel.transpose()*pod_direction.columnCopy(3);
    z(6) = podvel.transpose()*pod_direction_perp.columnCopy(3);

    return z + AdditiveNoiseMuGet();
}

}
