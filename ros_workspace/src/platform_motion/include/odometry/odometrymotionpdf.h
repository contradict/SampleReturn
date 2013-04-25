#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace platform_motion
{

class OdometryMotionPdf : public BFL::AnalyticConditionalGaussianAdditiveNoise
{
    public:
        /// Constructor
        /**
          @param additiveNoise Pdf representing the additive Gaussian uncertainty
          */
        OdometryMotionPdf( const BFL::Gaussian& processNoise );

        /// Destructor
        virtual ~OdometryMotionPdf();

        virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;
        virtual MatrixWrapper::ColumnVector ExpectedValueGet(void) const;

    private:

};

}
