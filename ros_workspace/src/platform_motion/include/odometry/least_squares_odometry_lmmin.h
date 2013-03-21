#include <odometry/lmmin.h>
#include <Eigen/Dense>


namespace platform_motion {

struct lmmin_data {
    Eigen::Vector2d body_pt;
    Eigen::Vector2d port_pos, port_dir;
    double port_delta;
    double port_vel;
    Eigen::Vector2d stern_pos, stern_dir;
    double stern_delta;
    double stern_vel;
    Eigen::Vector2d starboard_pos, starboard_dir;
    double starboard_delta;
    double starboard_vel;
    double interval;
    double min_translation_norm;
};

void lmmin_evaluate(const double *xytheta, int m_dat, const void *vdata, double *fvec, int *info);
}
