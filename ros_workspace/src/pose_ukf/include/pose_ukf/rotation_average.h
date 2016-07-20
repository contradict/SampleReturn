#include <sophus/so3.hpp>
#include <vector>
#pragma once

class RotationAverage
{
    std::vector<Sophus::SO3d> rotations;
    std::vector<double> weights;

    public:
        RotationAverage() :
            max_steps_(100),
            averaging_error_threshold_(1e-8)
        {};
        void operator()(double w, Sophus::SO3d r)
        {
            weights.push_back(w);
            rotations.push_back(r);
        };

        double geodesic(Sophus::SO3d *avg);
        double chordal(Sophus::SO3d *avg);

        int max_steps_;
        double averaging_error_threshold_;
};
