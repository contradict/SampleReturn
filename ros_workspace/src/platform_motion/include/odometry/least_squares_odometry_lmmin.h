#include <odometry/lmmin.h>
#include <Eigen/Dense>


namespace platform_motion {

void lmmin_evaluate(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info);
void lmmin_evaluate_velocity(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info);
}
