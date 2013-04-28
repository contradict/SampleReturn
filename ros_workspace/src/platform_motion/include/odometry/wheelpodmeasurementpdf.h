#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace platform_motion
{

class WheelPodMeasurementPdf : public BFL::AnalyticConditionalGaussianAdditiveNoise
{
    public:
        /// Constructor
        /**
          @param additiveNoise Pdf representing the additive Gaussian uncertainty
          */
        WheelPodMeasurementPdf( const BFL::Gaussian& measNoise, const MatrixWrapper::Matrix &pod_positions );

        /// Destructor
        virtual ~WheelPodMeasurementPdf();

        virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;
        virtual MatrixWrapper::ColumnVector ExpectedValueGet(void) const;

    private:
        MatrixWrapper::Matrix computePodDirections(void) const;

        MatrixWrapper::Matrix pod_positions;

};

}
