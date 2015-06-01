#include "odometry/odometry_node.h"

namespace platform_motion
{

class LLSOdometryNode: public OdometryNode
{
    public:
        LLSOdometryNode(void);
        virtual void init(void);

    private:
        virtual void computeOdometry(struct odometry_measurements &data, const ros::Time &stamp);

        double velocity_filter_;

};

}
