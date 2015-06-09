#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/internal.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

namespace cv
{
namespace linemod
{

/**
 * \brief Modality that computes quantized gradient orientations from a color image.
 * In contrast to the default color gradient modality, this finds gradients only
 * on the interior of the object.
 */
class InnerColorGradient : public Modality
{
public:
  /**
   * \brief Default constructor. Uses reasonable default parameter values.
   */
  InnerColorGradient();

  /**
   * \brief Constructor.
   *
   * \param weak_threshold   When quantizing, discard gradients with magnitude less than this.
   * \param num_features     How many features a template must contain.
   * \param strong_threshold Consider as candidate features only gradients whose norms are
   *                         larger than this.
   */
  InnerColorGradient(float weak_threshold, size_t num_features, float strong_threshold);

  virtual std::string name() const;

  virtual void read(const FileNode& fn);
  virtual void write(FileStorage& fs) const;

  float weak_threshold;
  size_t num_features;
  float strong_threshold;

protected:
  virtual Ptr<QuantizedPyramid> processImpl(const Mat& src,
                        const Mat& mask) const;
};

Ptr<Detector> getExpandedLINEMOD();

Ptr<Detector> getInnerLINE();

}
}
