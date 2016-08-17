#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <samplereturn_msgs/HueHistogram.h>

namespace saliency_detector
{

class HueHistogram;

class ColorModel
{
    protected:
    cv::Mat image_;
    cv::Mat mask_;

    void inner_mask(cv::Mat& mask) const;
    void outer_mask(cv::Mat& mask) const;

    HueHistogram
    createHueHistogram(const cv::Mat& mask, double min_color_saturation, int hbins) const;

    public:
    ColorModel() {};
    ColorModel(const cv::Mat& image, const cv::Mat& mask) :
        image_(image),
        mask_(mask)
    {};

    HueHistogram getInnerHueHistogram(double min_color_saturation, int hbins = 60) const;
    HueHistogram getOuterHueHistogram(double min_color_saturation, int hbins = 60) const;
    static HueHistogram getColoredSampleModel(std::vector<std::tuple<double, double>> edges, int hbins = 60);
};

class HueHistogram
{
    int hbins_;
    float min_color_saturation_;
    cv::Mat histogram_;

    HueHistogram(double min_color_saturation, int hbins=60) :
        hbins_(hbins),
        min_color_saturation_(min_color_saturation)
    {};

    public:
    HueHistogram(const samplereturn_msgs::HueHistogram &msg);

    double correlation(const HueHistogram& other);
    double intersection(const HueHistogram& other);
    double dominant_hue();
    void to_msg(samplereturn_msgs::HueHistogram* msg);

    friend class ColorModel;
};

}
