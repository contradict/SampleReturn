#include <saliency_detector/colormodel.h>

namespace saliency_detector
{

void
ColorModel::inner_mask(cv::Mat& mask) const
{
    cv::erode(mask_, mask, cv::Mat(), cv::Point(-1,-1), 2);
}

void
ColorModel::outer_mask(cv::Mat& mask) const
{
    cv::erode(255 - mask_, mask, cv::Mat());
}

HueHistogram
ColorModel::createHueHistogram(const cv::Mat& mask, double min_color_saturation, int hbins) const
{
    HueHistogram hh(min_color_saturation, hbins);
    cv::Mat image_hsv, combined_mask;
    cv::Mat saturation_mask, saturation;
    cv::cvtColor(image_, image_hsv, cv::COLOR_RGB2HSV);
    // Use mask to get color
    // and only consider mask where saturation is above threshold
    cv::extractChannel(image_hsv, saturation, 1);
    cv::threshold(saturation, saturation_mask, min_color_saturation, 255, cv::THRESH_BINARY);
    cv::bitwise_and(mask, saturation_mask, combined_mask);
    int histSize[] = { hbins };
    float hrange[] = { 0, 180 };
    const float* ranges[] = { hrange };
    int channels[] = { 0 };
    cv::calcHist(&image_hsv, 1, channels, combined_mask, hh.histogram_, 1,
            histSize, ranges , true, false);
    //Normalize histograms to account for different size
    cv::normalize(hh.histogram_, hh.histogram_, 1.0, 0.0, cv::NORM_MINMAX);
    return hh;
}

HueHistogram
ColorModel::getInnerHueHistogram(double min_color_saturation, int hbins) const
{
    cv::Mat mask;
    inner_mask(mask);
    return createHueHistogram(mask, min_color_saturation, hbins);
}

HueHistogram
ColorModel::getOuterHueHistogram(double min_color_saturation, int hbins) const
{
    cv::Mat mask;
    outer_mask(mask);
    return createHueHistogram(mask, min_color_saturation, hbins);
}

HueHistogram
ColorModel::getColoredSampleModel(std::vector<std::tuple<double, double>> edges, int hbins)
{
    HueHistogram hh(0, hbins);

    hh.histogram_ = cv::Mat(hbins, 1, CV_32F);
    hh.histogram_.setTo(cv::Scalar(0.0));
    for( auto& edge : edges )
    {
        hh.histogram_.rowRange(round(std::get<0>(edge)*hbins/180.0), round(std::get<1>(edge)*hbins/180.0)).setTo(cv::Scalar(1.0));
    }
    return hh;
}

HueHistogram::HueHistogram(const samplereturn_msgs::HueHistogram& msg) :
    hbins_(msg.hbins),
    min_color_saturation_(msg.min_color_saturation),
    histogram_(msg.histogram)
{
}

double
HueHistogram::correlation(const HueHistogram& other)
{
    double c = cv::compareHist(histogram_, other.histogram_, cv::HISTCMP_CORREL);
    return c;
}

double
HueHistogram::intersection(const HueHistogram& other)
{
    double c = cv::compareHist(histogram_, other.histogram_, cv::HISTCMP_INTERSECT);
    return c;
}


double
HueHistogram::dominant_hue()
{
    double min_hue, max_hue;
    int min_hue_loc, max_hue_loc;
    cv::minMaxIdx(histogram_, &min_hue, &max_hue, &min_hue_loc, &max_hue_loc);
    double dominant_hue = max_hue_loc * (180/histogram_.rows);
    return dominant_hue;
}

void
HueHistogram::to_msg(samplereturn_msgs::HueHistogram* msg)
{
    msg->hbins = hbins_;
    msg->min_color_saturation = min_color_saturation_;
    for(int i=0;i<histogram_.rows;i++)
    {
        msg->histogram.push_back(histogram_.at<float>(i));
    }
}

}
