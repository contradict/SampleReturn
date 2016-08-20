#include <opencv2/opencv.hpp>

namespace linemod_detector {

bool computeGripAngle(const cv::Mat& mask, cv::RotatedRect* griprect);
bool computeBoundingBox(const cv::Mat& mask, cv::Rect* rect);

}

