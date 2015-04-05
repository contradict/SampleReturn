#include <pose_ukf/rotation_average.h>
#include <pose_ukf/zip_range.hpp>

double
RotationAverage::geodesic(Sophus::SO3d *avg)
{
    *avg=rotations.front();
    int steps=max_steps_;
    while(steps>0)
    {
        steps--;
        Eigen::Vector3d r(Eigen::Vector3d::Zero());
        double wsum=0;
        for(auto && t: zip_range(weights, rotations))
        {
            double w=t.get<0>();
            const Sophus::SO3d& q = t.get<1>();
            r += w*((*avg).inverse()*q).log();
            wsum += w;
        }
        r /= wsum;
        if(r.norm()<averaging_error_threshold_)
        {
            return r.norm();
        }
        (*avg) = (*avg)*Sophus::SO3d::exp(r);
    }

    return -1;
}

double
RotationAverage::chordal(Sophus::SO3d *avg)
{
    double max_angle=0;
    Eigen::Matrix4d Q;
    Q.setZero();
    auto rest = rotations.begin();
    rest++;
    for(auto && t: zip_range(weights, rotations))
    {
        double w=t.get<0>();
        Sophus::SO3d& q = t.get<1>();
        Q += w * q.unit_quaternion().coeffs() * q.unit_quaternion().coeffs().transpose();
        max_angle = std::accumulate(rest, rotations.end(), max_angle,
                [&q](double max, const Sophus::SO3d& other)
                {
                return std::max(max,
                    q.unit_quaternion().angularDistance(other.unit_quaternion()));
                });
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
    solver.compute(Q);
    Eigen::Vector4d::Map(avg->data()) = solver.eigenvectors().col(3);
    return max_angle;
 }
