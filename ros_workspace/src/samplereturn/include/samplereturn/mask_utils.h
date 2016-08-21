#include <opencv2/opencv.hpp>

namespace linemod_detector {

bool computeGripAngle(const cv::Mat& mask, cv::RotatedRect* griprect, float *grip_angle);
bool computeBoundingBox(const cv::Mat& mask, cv::Rect* rect);
void drawGripRect(cv::Mat& debug_image, const cv::RotatedRect& griprect);

}
