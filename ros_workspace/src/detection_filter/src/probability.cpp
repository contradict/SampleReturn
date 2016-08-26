#include <limits>
#include <algorithm>

namespace detection_filter
{

double updateProb(double PO, bool detection, double PDgO, double PDgo)
{
    if (detection) {
        // Probability of a detection
        double PD = PDgO * PO + PDgo * (1 - PO);
        // Update
        PO = PDgO * PO / PD;
    }
    else {
        // Probability of no detection
        double Pd = (1 - PDgO) * PO + (1.0 - PDgo) * (1.0 - PO);
        // Update
        PO = (1.0 - PDgO) * PO / Pd;
    }
    return std::max(
            std::min(PO, (1.0-std::numeric_limits<double>::epsilon())),
                std::numeric_limits<double>::epsilon());
}

}
