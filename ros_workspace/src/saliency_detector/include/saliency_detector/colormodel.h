#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <samplereturn_msgs/HueHistogram.h>

namespace saliency_detector
{

class HueHistogram;

class HueHistogramExemplar;

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
    static HueHistogramExemplar getColoredSampleModel(std::vector<std::tuple<double, double>> edges, int hbins = 60);
    static HueHistogramExemplar getValuedSampleModel(int hbins = 60);
};

class HueHistogram
{
    int hbins_;
    double min_color_saturation_;
    double saturation_score_, value_mean_;
    cv::Mat histogram_;

    protected:
    HueHistogram(double min_color_saturation, int hbins=60) :
        hbins_(hbins),
        min_color_saturation_(min_color_saturation)
    {};

    public:
    HueHistogram(const samplereturn_msgs::HueHistogram &msg);

    double correlation(const HueHistogram& other) const;
    double intersection(const HueHistogram& other) const;
    double dominant_hue() const;
    void to_msg(samplereturn_msgs::HueHistogram* msg) const;

    virtual double
    distance(const HueHistogram& other, double low_saturation, double high_saturation) const;
    virtual double
    distance(const HueHistogramExemplar& other, double low_saturation, double high_saturation) const;

    char * str() const;
    void draw_histogram(cv::Mat image, int x, int y) const;

    friend class ColorModel;
};

class HueHistogramExemplar : public HueHistogram
{
    HueHistogramExemplar(double min_color_saturation, int hbins = 60) :
        HueHistogram(min_color_saturation, hbins)
    {};
    public:
    virtual double
    distance(const HueHistogram& other, double low_saturation, double high_saturation) const;
    friend class ColorModel;
};

}
