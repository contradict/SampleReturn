#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdio.h>
#include <fstream>

/* Load color matrix
 * Compute index
 * Compute interior color
 * Compute exterior color
 */

class ColorNaming
{
  public:
    ColorNaming();
    //ColorNaming(std::string filename);
    int computeIndex(cv::Vec3b pixel);
    cv::Mat computeIndexImage(cv::Mat image);
    Eigen::Matrix<float,11,1> computeInteriorColor(cv::Mat image, cv::Mat mask);
    Eigen::Matrix<float,11,1> computeExteriorColor(cv::Mat image, cv::Mat mask);
  private:
    float color_index_array_ [458752];
};
