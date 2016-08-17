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
ColorModel::createHueHistogram(const cv::Mat& mask, double min_color_saturation, double ll, double hl, int hbins) const
{
    HueHistogram hh(min_color_saturation, ll, hl, hbins);
    cv::Mat image_hsv, combined_mask;
    cv::Mat saturation_mask, saturation, value;
    cv::cvtColor(image_, image_hsv, cv::COLOR_RGB2HSV);
    // Use mask to get color
    // and only consider mask where saturation is above threshold
    cv::extractChannel(image_hsv, saturation, 1);
    cv::extractChannel(image_hsv, value, 2);
    cv::threshold(saturation, saturation_mask, min_color_saturation, 255, cv::THRESH_BINARY);
    cv::bitwise_and(mask, saturation_mask, combined_mask);
    double salient_pixels = cv::countNonZero(mask);
    hh.saturation_score_ = double(cv::countNonZero(combined_mask))/salient_pixels;
    hh.value_mean_ = cv::mean(value, mask)[0];
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
ColorModel::getInnerHueHistogram(double min_color_saturation, double ll, double hl, int hbins) const
{
    cv::Mat mask;
    inner_mask(mask);
    return createHueHistogram(mask, min_color_saturation, ll, hl, hbins);
}

HueHistogram
ColorModel::getOuterHueHistogram(double min_color_saturation, double ll, double hl, int hbins) const
{
    cv::Mat mask;
    outer_mask(mask);
    return createHueHistogram(mask, min_color_saturation, ll, hl, hbins);
}

HueHistogramExemplar
ColorModel::getColoredSampleModel(std::vector<std::tuple<double, double>> edges, double ll, double hl, int hbins)
{
    HueHistogramExemplar hh(0, ll, hl, hbins);

    hh.saturation_score_ = 1.0;
    hh.value_mean_ = 128.0;
    hh.low_saturation_limit_ = ll;
    hh.high_saturation_limit_ = hl;
    hh.histogram_ = cv::Mat(hbins, 1, CV_32F);
    hh.histogram_.setTo(cv::Scalar(0.0));
    for( auto& edge : edges )
    {
        hh.histogram_.rowRange(round(std::get<0>(edge)*hbins/180.0), round(std::get<1>(edge)*hbins/180.0)).setTo(cv::Scalar(1.0));
    }
    return hh;
}

HueHistogramExemplar
ColorModel::getValuedSampleModel(double ll, double hl, int hbins)
{
    HueHistogramExemplar hh(0, ll, hl, hbins);

    hh.saturation_score_ = 0.0;
    hh.value_mean_ = 255.0;
    hh.low_saturation_limit_ = ll;
    hh.high_saturation_limit_ = hl;
    hh.histogram_ = cv::Mat(hbins, 1, CV_32F);
    hh.histogram_.setTo(cv::Scalar(0.0));
    return hh;
}

HueHistogram::HueHistogram(const samplereturn_msgs::HueHistogram& msg) :
    hbins_(msg.hbins),
    min_color_saturation_(msg.min_color_saturation),
    saturation_score_(msg.saturation_score),
    value_mean_(msg.value_mean),
    low_saturation_limit_(msg.low_saturation_limit),
    high_saturation_limit_(msg.high_saturation_limit),
    histogram_(msg.histogram)
{
}

double
HueHistogram::correlation(const HueHistogram& other) const
{
    double c = cv::compareHist(histogram_, other.histogram_, cv::HISTCMP_CORREL);
    return c;
}

double
HueHistogram::intersection(const HueHistogram& other) const
{
    double c = cv::compareHist(histogram_, other.histogram_, cv::HISTCMP_INTERSECT);
    return c;
}

double
HueHistogram::distance(const HueHistogram& other) const
{
    if( (this->saturation_score_<low_saturation_limit_) && (other.saturation_score_<low_saturation_limit_) ) {
        return fabs(double(this->value_mean_ - other.value_mean_)/std::max(this->value_mean_, other.value_mean_));
    }
    else if( (this->saturation_score_>high_saturation_limit_) && (other.saturation_score_>high_saturation_limit_) )
    {
        return 1.0 - correlation(other);
    }
    else
    {
        return 1.0;
    }
}

double
HueHistogram::distance(const HueHistogramExemplar& other) const
{
    if( (this->saturation_score_<low_saturation_limit_) && (other.saturation_score_<low_saturation_limit_) )
    {
        return 1.0-(other.value_mean_ - this->value_mean_)/other.value_mean_;
    }
    else if( (this->saturation_score_>high_saturation_limit_) && (other.saturation_score_>high_saturation_limit_) )
    {
        return intersection(other);
    }
    else
    {
        return 1.0;
    }
}

double
HueHistogram::dominant_hue() const
{
    double min_hue, max_hue;
    int min_hue_loc, max_hue_loc;
    cv::minMaxIdx(histogram_, &min_hue, &max_hue, &min_hue_loc, &max_hue_loc);
    double dominant_hue = max_hue_loc * (180/histogram_.rows);
    return dominant_hue;
}

void
HueHistogram::to_msg(samplereturn_msgs::HueHistogram* msg) const
{
    msg->hbins = hbins_;
    msg->min_color_saturation = min_color_saturation_;
    msg->saturation_score = saturation_score_;
    msg->value_mean = value_mean_;
    msg->low_saturation_limit = low_saturation_limit_;
    msg->high_saturation_limit = high_saturation_limit_;
    for(int i=0;i<histogram_.rows;i++)
    {
        msg->histogram.push_back(histogram_.at<float>(i));
    }
}

char *
HueHistogram::str() const
{
    char *string=NULL;
    int len = asprintf(&string, "H:%3.0f S:%04.3f V:%3.0f", dominant_hue(),
                 saturation_score_, value_mean_);
    if(len<0)
    {
        string = (char *)calloc(1,1);
    }
    return string;
}

void
HueHistogram::draw_histogram(cv::Mat image, int x, int y) const
{
    const int height_scale = 50;
    int y_spark = y + height_scale + 2;
    std::vector<cv::Point> points;
    for(int j=0;j<histogram_.rows; j++)
    {
        points.push_back(cv::Point(x+j, y_spark-round(histogram_.at<float>(j)*height_scale)));
    }
    const cv::Point *pts = (const cv::Point*) cv::Mat(points).data;
    int npts = cv::Mat(points).rows;
    cv::polylines(image, &pts, &npts, 1, false, cv::Scalar(255,0,0), 3, CV_AA, 0);
}

double
HueHistogramExemplar::distance(const HueHistogram& other) const
{
    return other.distance(*this);
}

}
