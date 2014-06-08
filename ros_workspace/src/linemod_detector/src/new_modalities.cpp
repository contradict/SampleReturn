#include "new_modalities.hpp"
//#include "precomp.hpp"
//#include <limits>

namespace cv
{
namespace linemod
{

static const int T_DEFAULTS[] = {5, 8};
static const int T_DEFAULTS_NEW[] = {3, 3};

/**
 * \brief Get the label [0,8) of the single bit set in quantized.
 */
static inline int getLabel(int quantized)
{
  switch (quantized)
  {
    case 1:   return 0;
    case 2:   return 1;
    case 4:   return 2;
    case 8:   return 3;
    case 16:  return 4;
    case 32:  return 5;
    case 64:  return 6;
    case 128: return 7;
    default:
      CV_Error(CV_StsBadArg, "Invalid value of quantized parameter");
      return -1; //avoid warning
  }
}
// Forward declaration
void hysteresisGradient(Mat& magnitude, Mat& angle,
                        Mat& ap_tmp, float threshold);

/**
 * \brief Compute quantized orientation image from color image.
 *
 * Implements section 2.2 "Computing the Gradient Orientations."
 *
 * \param[in]  src       The source 8-bit, 3-channel image.
 * \param[out] magnitude Destination floating-point array of squared magnitudes.
 * \param[out] angle     Destination 8-bit array of orientations. Each bit
 *                       represents one bin of the orientation space.
 * \param      threshold Magnitude threshold. Keep only gradients whose norms are
 *                       larger than this.
 */
static void quantizedOrientations(const Mat& src, Mat& magnitude,
                           Mat& angle, float threshold)
{
  magnitude.create(src.size(), CV_32F);

  // Allocate temporary buffers
  Size size = src.size();
  Mat sobel_3dx; // per-channel horizontal derivative
  Mat sobel_3dy; // per-channel vertical derivative
  Mat sobel_dx(size, CV_32F);      // maximum horizontal derivative
  Mat sobel_dy(size, CV_32F);      // maximum vertical derivative
  Mat sobel_ag;  // final gradient orientation (unquantized)
  Mat smoothed;

  // Compute horizontal and vertical image derivatives on all color channels separately
  static const int KERNEL_SIZE = 7;
  // For some reason cvSmooth/cv::GaussianBlur, cvSobel/cv::Sobel have different defaults for border handling...
  GaussianBlur(src, smoothed, Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0, BORDER_REPLICATE);
  Sobel(smoothed, sobel_3dx, CV_16S, 1, 0, 3, 1.0, 0.0, BORDER_REPLICATE);
  Sobel(smoothed, sobel_3dy, CV_16S, 0, 1, 3, 1.0, 0.0, BORDER_REPLICATE);

  short * ptrx  = (short *)sobel_3dx.data;
  short * ptry  = (short *)sobel_3dy.data;
  float * ptr0x = (float *)sobel_dx.data;
  float * ptr0y = (float *)sobel_dy.data;
  float * ptrmg = (float *)magnitude.data;

  const int length1 = static_cast<const int>(sobel_3dx.step1());
  const int length2 = static_cast<const int>(sobel_3dy.step1());
  const int length3 = static_cast<const int>(sobel_dx.step1());
  const int length4 = static_cast<const int>(sobel_dy.step1());
  const int length5 = static_cast<const int>(magnitude.step1());
  const int length0 = sobel_3dy.cols * 3;

  for (int r = 0; r < sobel_3dy.rows; ++r)
  {
    int ind = 0;

    for (int i = 0; i < length0; i += 3)
    {
      // Use the gradient orientation of the channel whose magnitude is largest
      int mag1 = CV_SQR(ptrx[i]) + CV_SQR(ptry[i]);
      int mag2 = CV_SQR(ptrx[i + 1]) + CV_SQR(ptry[i + 1]);
      int mag3 = CV_SQR(ptrx[i + 2]) + CV_SQR(ptry[i + 2]);

      if (mag1 >= mag2 && mag1 >= mag3)
      {
        ptr0x[ind] = ptrx[i];
        ptr0y[ind] = ptry[i];
        ptrmg[ind] = (float)mag1;
      }
      else if (mag2 >= mag1 && mag2 >= mag3)
      {
        ptr0x[ind] = ptrx[i + 1];
        ptr0y[ind] = ptry[i + 1];
        ptrmg[ind] = (float)mag2;
      }
      else
      {
        ptr0x[ind] = ptrx[i + 2];
        ptr0y[ind] = ptry[i + 2];
        ptrmg[ind] = (float)mag3;
      }
      ++ind;
    }
    ptrx += length1;
    ptry += length2;
    ptr0x += length3;
    ptr0y += length4;
    ptrmg += length5;
  }

  // Calculate the final gradient orientations
  phase(sobel_dx, sobel_dy, sobel_ag, true);
  hysteresisGradient(magnitude, angle, sobel_ag, CV_SQR(threshold));
}

void hysteresisGradient(Mat& magnitude, Mat& quantized_angle,
                        Mat& angle, float threshold)
{
  // Quantize 360 degree range of orientations into 16 buckets
  // Note that [0, 11.25), [348.75, 360) both get mapped in the end to label 0,
  // for stability of horizontal and vertical features.
  Mat_<unsigned char> quantized_unfiltered;
  angle.convertTo(quantized_unfiltered, CV_8U, 16.0 / 360.0);

  // Zero out top and bottom rows
  /// @todo is this necessary, or even correct?
  memset(quantized_unfiltered.ptr(), 0, quantized_unfiltered.cols);
  memset(quantized_unfiltered.ptr(quantized_unfiltered.rows - 1), 0, quantized_unfiltered.cols);
  // Zero out first and last columns
  for (int r = 0; r < quantized_unfiltered.rows; ++r)
  {
    quantized_unfiltered(r, 0) = 0;
    quantized_unfiltered(r, quantized_unfiltered.cols - 1) = 0;
  }

  // Mask 16 buckets into 8 quantized orientations
  for (int r = 1; r < angle.rows - 1; ++r)
  {
    uchar* quant_r = quantized_unfiltered.ptr<uchar>(r);
    for (int c = 1; c < angle.cols - 1; ++c)
    {
      quant_r[c] &= 7;
    }
  }

  // Filter the raw quantized image. Only accept pixels where the magnitude is above some
  // threshold, and there is local agreement on the quantization.
  quantized_angle = Mat::zeros(angle.size(), CV_8U);
  for (int r = 1; r < angle.rows - 1; ++r)
  {
    float* mag_r = magnitude.ptr<float>(r);

    for (int c = 1; c < angle.cols - 1; ++c)
    {
      if (mag_r[c] > threshold)
      {
  // Compute histogram of quantized bins in 3x3 patch around pixel
        int histogram[8] = {0, 0, 0, 0, 0, 0, 0, 0};

        uchar* patch3x3_row = &quantized_unfiltered(r-1, c-1);
        histogram[patch3x3_row[0]]++;
        histogram[patch3x3_row[1]]++;
        histogram[patch3x3_row[2]]++;

  patch3x3_row += quantized_unfiltered.step1();
        histogram[patch3x3_row[0]]++;
        histogram[patch3x3_row[1]]++;
        histogram[patch3x3_row[2]]++;

  patch3x3_row += quantized_unfiltered.step1();
        histogram[patch3x3_row[0]]++;
        histogram[patch3x3_row[1]]++;
        histogram[patch3x3_row[2]]++;

  // Find bin with the most votes from the patch
        int max_votes = 0;
        int index = -1;
        for (int i = 0; i < 8; ++i)
        {
          if (max_votes < histogram[i])
          {
            index = i;
            max_votes = histogram[i];
          }
        }

  // Only accept the quantization if majority of pixels in the patch agree
  static const int NEIGHBOR_THRESHOLD = 5;
        if (max_votes >= NEIGHBOR_THRESHOLD)
          quantized_angle.at<uchar>(r, c) = uchar(1 << index);
      }
    }
  }
}

class InnerColorGradientPyramid : public QuantizedPyramid
{
public:
  InnerColorGradientPyramid(const Mat& src, const Mat& mask,
                       float weak_threshold, size_t num_features,
                       float strong_threshold);

  virtual void quantize(Mat& dst) const;

  virtual bool extractTemplate(Template& templ) const;

  virtual void pyrDown();

protected:
  /// Recalculate angle and magnitude images
  void update();

  Mat src;
  Mat mask;

  int pyramid_level;
  Mat angle;
  Mat magnitude;

  float weak_threshold;
  size_t num_features;
  float strong_threshold;
};

InnerColorGradientPyramid::InnerColorGradientPyramid(const Mat& _src, const Mat& _mask,
                                           float _weak_threshold, size_t _num_features,
                                           float _strong_threshold)
  : src(_src),
    mask(_mask),
    pyramid_level(0),
    weak_threshold(_weak_threshold),
    num_features(_num_features),
    strong_threshold(_strong_threshold)
{
  update();
}

void InnerColorGradientPyramid::update()
{
  quantizedOrientations(src, magnitude, angle, weak_threshold);
}

void InnerColorGradientPyramid::pyrDown()
{
  // Some parameters need to be adjusted
  num_features /= 2; /// @todo Why not 4?
  ++pyramid_level;

  // Downsample the current inputs
  Size size(src.cols / 2, src.rows / 2);
  Mat next_src;
  cv::pyrDown(src, next_src, size);
  src = next_src;
  if (!mask.empty())
  {
    Mat next_mask;
    resize(mask, next_mask, size, 0.0, 0.0, CV_INTER_NN);
    mask = next_mask;
  }

  update();
}

void InnerColorGradientPyramid::quantize(Mat& dst) const
{
  dst = Mat::zeros(angle.size(), CV_8U);
  angle.copyTo(dst, mask);
}

bool InnerColorGradientPyramid::extractTemplate(Template& templ) const
{
  // Want internal features, not especially interested in outer shape
  Mat local_mask;
  if (!mask.empty())
  {
    Mat element = getStructuringElement(MORPH_RECT, Size(31,31));
    erode(mask, local_mask, element, Point(-1,-1), 1, BORDER_REPLICATE);
  }

  // Create sorted list of all pixels with magnitude greater than a threshold
  std::vector<Candidate> candidates;
  bool no_mask = local_mask.empty();
  float threshold_sq = CV_SQR(strong_threshold);
  for (int r = 0; r < magnitude.rows; ++r)
  {
    const uchar* angle_r = angle.ptr<uchar>(r);
    const float* magnitude_r = magnitude.ptr<float>(r);
    const uchar* mask_r = no_mask ? NULL : local_mask.ptr<uchar>(r);

    for (int c = 0; c < magnitude.cols; ++c)
    {
      if (no_mask || mask_r[c])
      {
        uchar quantized = angle_r[c];
        if (quantized > 0)
        {
          float score = magnitude_r[c];
          if (score > threshold_sq)
          {
            candidates.push_back(Candidate(c, r, getLabel(quantized), score));
          }
        }
      }
    }
  }
  // We require a certain number of features
  if (candidates.size() < num_features)
    return false;
  // NOTE: Stable sort to agree with old code, which used std::list::sort()
  std::stable_sort(candidates.begin(), candidates.end());

  // Use heuristic based on surplus of candidates in narrow outline for initial distance threshold
  float distance = static_cast<float>(candidates.size() / num_features + 1);
  selectScatteredFeatures(candidates, templ.features, num_features, distance);

  // Size determined externally, needs to match templates for other modalities
  templ.width = -1;
  templ.height = -1;
  templ.pyramid_level = pyramid_level;

  return true;
}

InnerColorGradient::InnerColorGradient()
  : weak_threshold(10.0f),
    num_features(63),
    strong_threshold(55.0f)
{
}

InnerColorGradient::InnerColorGradient(float _weak_threshold, size_t _num_features, float _strong_threshold)
  : weak_threshold(_weak_threshold),
    num_features(_num_features),
    strong_threshold(_strong_threshold)
{
}

static const char CG_NAME[] = "InnerColorGradient";

std::string InnerColorGradient::name() const
{
  return CG_NAME;
}

Ptr<QuantizedPyramid> InnerColorGradient::processImpl(const Mat& src,
                                                     const Mat& mask) const
{
  return new InnerColorGradientPyramid(src, mask, weak_threshold, num_features, strong_threshold);
}

void InnerColorGradient::read(const FileNode& fn)
{
  std::string type = fn["type"];
  CV_Assert(type == CG_NAME);

  weak_threshold = fn["weak_threshold"];
  num_features = int(fn["num_features"]);
  strong_threshold = fn["strong_threshold"];
}

void InnerColorGradient::write(FileStorage& fs) const
{
  fs << "type" << CG_NAME;
  fs << "weak_threshold" << weak_threshold;
  fs << "num_features" << int(num_features);
  fs << "strong_threshold" << strong_threshold;
}

Ptr<Detector> getExpandedLINEMOD()
{
  std::vector< Ptr<Modality> > modalities;
  modalities.push_back(new ColorGradient);
  modalities.push_back(new DepthNormal);
  modalities.push_back(new InnerColorGradient);
  return new Detector(modalities, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
}

Ptr<Detector> getInnerLINE()
{
  std::vector< Ptr<Modality> > modalities;
  modalities.push_back(new InnerColorGradient);
  return new Detector(modalities, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
}

} // namespace linemod
} // namespace cv
