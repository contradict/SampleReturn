#include "color_naming.h"

ColorNaming::ColorNaming()
{
  std::ifstream infile("w2c.txt");
  //For the full version, 32768*14=458752
  if(infile.is_open())
  {
    for(int i=0; i<458752; i++)
    {
      infile >> color_index_array_[i];
    }
  }
}

//ColorNaming::ColorNaming(std::string filename)
//{
//  std::ifstream infile(filename);
//  //For the full version, 32768*14=458752
//  if(infile.is_open())
//  {
//    for(int i=0; i<458752; i++)
//    {
//      infile >> color_index_array_[i];
//    }
//  }
//}

int ColorNaming::computeIndex(cv::Vec3b pixel)
{
  int index = floor(pixel[0]/8.) + 32*floor(pixel[1]/8.) + 32*32*floor(pixel[2]/8.);
  return index;
}

cv::Mat ColorNaming::computeIndexImage(cv::Mat image)
{
  cv::Mat index_image(image.rows, image.cols, CV_16UC1);
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      index_image.at<unsigned short>(i,j) = computeIndex(image.at<cv::Vec3b>(i,j));
    }
  }
  return index_image;
}

Eigen::Matrix<float,11,1> ColorNaming::computeInteriorColor(cv::Mat image, cv::Mat mask)
{
  cv::Mat index_image = computeIndexImage(image);
  cv::Mat eroded_mask;
  Eigen::Matrix<float,11,1> cum_color_score;
  cv::erode(mask, eroded_mask, cv::Mat());
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      if (eroded_mask.at<unsigned short>(i,j) != 0) {
        int index = 14*index_image.at<unsigned short>(i,j);
        Eigen::Matrix<float,11,1> color_score;
        for (int k=3; k<14; k++) {
          color_score[k-3] = color_index_array_[k+index];
        }
        cum_color_score += color_score;
      }
    }
  }
  cum_color_score.normalize();
  return cum_color_score;
}

Eigen::Matrix<float,11,1> ColorNaming::computeExteriorColor(cv::Mat image, cv::Mat mask)
{
  cv::Mat index_image = computeIndexImage(image);
  cv::Mat dilated_mask;
  Eigen::Matrix<float,11,1> cum_color_score;
  cv::dilate(mask, dilated_mask, cv::Mat());
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      if (dilated_mask.at<unsigned short>(i,j) == 0) {
        int index = 14*index_image.at<unsigned short>(i,j);
        Eigen::Matrix<float,11,1> color_score;
        for (int k=3; k<14; k++) {
          color_score[k-3] = color_index_array_[k+index];
        }
        cum_color_score += color_score;
      }
    }
  }
  cum_color_score.normalize();
  return cum_color_score;
}
