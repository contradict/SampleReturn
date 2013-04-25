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
    MatrixWrapper::Matrix df(6,6);

    if(i==0)
    {
        //[[ dv0/dx       dv0/dy       dv0/dtheta       dv0/dvx         dv0/dvy         dv0/domega]
        // [ dv0_perp/dx  dv0_perp/dy  dv0_perp/dtheta  dv0_perp/dvx    dv0_perp/dvy    dv0_perp/domega]
        // [ dv1/dx       dv1/dy       dv1/dtheta       dv1/dvx         dv1/dvy         dv1/domega]
        // [ dv1_perp_dx  dv1_perp_dy  dv1_perp_dtheta  dv1_perp/dvx    dv1_perp/dvy    dv1_perp/domega]
        // [ dv2/dx       dv2/dy       dv2/dtheta       dv2/dvx         dv2/dvy         dv2/domega]]
        // [ dv2_perp/dx  dv2_perp/dy  dv2_perp/dtheta  dv2_perp/dvx    dv2_perp/dvy    dv2_perp/domega]]
        df      = 0;
        df(1,4) =      pod_direction(1,1);
        df(1,5) =      pod_direction(2,1);
        df(1,6) = -1.0*pod_positions(2,1)*pod_direction(1,1)
                  +    pod_positions(1,1)*pod_direction(2,1);

        df(2,4) = -1.0*pod_direction(2,1); 
        df(2,5) =      pod_direction(1,1); 
        df(2,6) =      pod_positions(2,1)*pod_direction(2,1)
                  +    pod_positions(1,1)*pod_direction(1,1);

        df(3,4) =      pod_direction(1,2);
        df(3,5) =      pod_direction(2,2);
        df(3,6) = -1.0*pod_positions(2,2)*pod_direction(1,2)
                  +    pod_positions(1,2)*pod_direction(2,2);

        df(4,4) = -1.0*pod_direction(2,2); 
        df(4,5) =      pod_direction(1,2); 
        df(4,6) =      pod_positions(2,2)*pod_direction(2,2)
                  +    pod_positions(1,2)*pod_direction(1,2);

        df(5,4) = pod_direction(1,3);
        df(5,5) = pod_direction(2,3);
        df(5,6) = -1*pod_positions(2,3)*pod_direction(1,3)
                  +  pod_positions(1,3)*pod_direction(2,3);

        df(6,4) = -1.0*pod_direction(2,3); 
        df(6,5) =      pod_direction(1,3); 
        df(6,6) =      pod_positions(2,3)*pod_direction(2,3)
                  +    pod_positions(1,3)*pod_direction(1,3);
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
    MatrixWrapper::ColumnVector z(6);

    MatrixWrapper::ColumnVector vel(3);
    vel(1) = state(4);
    vel(2) = state(5);
    vel(3) = 0;

    MatrixWrapper::Matrix zCross(3,3);
    zCross = 0;
    zCross(1, 2) = -1;
    zCross(2, 1) =  1;

    MatrixWrapper::Matrix pod_direction = computePodDirections();
    MatrixWrapper::Matrix pod_direction_perp = zCross*pod_direction;

    MatrixWrapper::Matrix pod_circular_vel = zCross*pod_positions*state(6);

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
