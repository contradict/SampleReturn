
#include "odometry/odometry_node.h"

namespace platform_motion
{

class LeastSquaresOdometryNode: public OdometryNode
{
    public:
        LeastSquaresOdometryNode(void);

    private:

        virtual void computeOdometry(struct odometry_measurements &data, const ros::Time &stamp);

        bool leastsquares_odometry;
        bool leastsquares_velocity;
};

}
