#include "color_naming.h"

ColorNaming::ColorNaming()
{
  std::string path = ros::package::getPath("saliency_detector");
  path = path + "/src/w2c.txt";
  std::ifstream infile(path.c_str());
  //For the full version, 32768*14=458752
  if(infile.is_open())
  {
    for(int i=0; i<458752; i++)
    {
      infile >> color_index_array_[i];
    }
  }
}

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
  ROS_INFO("Sub image size: %i, %i", image.size().width, image.size().height);
  ROS_INFO("Sub mask size: %i, %i", mask.size().width, mask.size().height);
  cv::Mat index_image = computeIndexImage(image);
  cv::Mat eroded_mask;
  Eigen::Matrix<float,11,1> sum_color_score(Eigen::Matrix<float,11,1>::Zero());
  //cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9,9));
  cv::erode(mask, eroded_mask, cv::Mat(), cv::Point(-1,-1), 1);
  //cv::erode(mask, eroded_mask, element);
  //eroded_mask = mask.clone();
  int count = 0;
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      if (eroded_mask.at<unsigned short>(i,j) != 0) {
        count += 1;
        int index = 14*index_image.at<unsigned short>(i,j);
        Eigen::Matrix<float,11,1> color_score;
        for (int k=3; k<14; k++) {
          color_score[k-3] = color_index_array_[k+index];
        }
        sum_color_score += color_score;
      }
    }
  }
  ROS_INFO("Interior Count: %i", count);
  sum_color_score /= count;
  ROS_INFO("Interio Sum: %f", sum_color_score.sum());
  return sum_color_score;
}

Eigen::Matrix<float,11,1> ColorNaming::computeInteriorColor(cv::Mat image,
    cv::Mat mask, Eigen::Matrix<float,11,1> exteriorColor)
{
  ROS_INFO("Sub image size: %i, %i", image.size().width, image.size().height);
  ROS_INFO("Sub mask size: %i, %i", mask.size().width, mask.size().height);
  cv::Mat index_image = computeIndexImage(image);
  cv::Mat eroded_mask;
  Eigen::Matrix<float,11,1> sum_color_score(Eigen::Matrix<float,11,1>::Zero());
  cv::erode(mask, eroded_mask, cv::Mat(), cv::Point(-1,-1), 1);
  int count = 0;
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      if (eroded_mask.at<unsigned short>(i,j) != 0) {
        int index = 14*index_image.at<unsigned short>(i,j);
        Eigen::Matrix<float,11,1> color_score;
        for (int k=3; k<14; k++) {
          color_score[k-3] = color_index_array_[k+index];
        }
        Eigen::Array<float,11,1> diff = (color_score-exteriorColor);
        diff = diff.abs();
        if (diff.sum() >= 0.4) {
          count += 1;
          sum_color_score += color_score;
        }
      }
    }
  }
  ROS_INFO("Interior Count: %i", count);
  if (count != 0) {
    sum_color_score /= count;
  }
  ROS_INFO("Interior Sum: %f", sum_color_score.sum());
  return sum_color_score;
}

Eigen::Matrix<float,11,1> ColorNaming::computeExteriorColor(cv::Mat image, cv::Mat mask)
{
  cv::Mat index_image = computeIndexImage(image);
  cv::Mat dilated_mask;
  Eigen::Matrix<float,11,1> sum_color_score(Eigen::Matrix<float,11,1>::Zero());
  cv::dilate(mask, dilated_mask, cv::Mat());
  int count = 0;
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      if (dilated_mask.at<unsigned short>(i,j) == 0) {
        count += 1;
        int index = 14*index_image.at<unsigned short>(i,j);
        Eigen::Matrix<float,11,1> color_score;
        for (int k=3; k<14; k++) {
          color_score[k-3] = color_index_array_[k+index];
        }
        sum_color_score += color_score;
      }
    }
  }
  ROS_INFO("Exterior Count: %i", count);
  std::cout << sum_color_score.sum() << std::endl;
  sum_color_score /= count;
  std::cout << sum_color_score.sum() << std::endl;
  return sum_color_score;
}

std::string ColorNaming::getDominantColor(Eigen::Matrix<float,11,1> color_score)
{
  std::string names[11] = {"black","blue","brown","gray","green","orange","pink","purple","red","white","yellow"};
  int maxIndex;
  color_score.maxCoeff(&maxIndex);
  return names[maxIndex];
}

int ColorNaming::computeInteriorColorMax(cv::Mat image, cv::Mat mask)
{
  ROS_INFO("Sub image size: %i, %i", image.size().width, image.size().height);
  ROS_INFO("Sub mask size: %i, %i", mask.size().width, mask.size().height);
  cv::Mat index_image = computeIndexImage(image);
  cv::Mat eroded_mask;
  Eigen::Matrix<float,11,1> sum_color_score(Eigen::Matrix<float,11,1>::Zero());
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,1));
  cv::erode(mask, eroded_mask, element, cv::Point(-1,-1), 2);
  int count = 0;
  for (int i=0; i<image.rows; i++) {
    for (int j=0; j<image.cols; j++) {
      if (eroded_mask.at<unsigned short>(i,j) != 0) {
        count += 1;
        int index = 14*index_image.at<unsigned short>(i,j);
        Eigen::Matrix<float,11,1> color_score;
        for (int k=3; k<14; k++) {
          color_score[k-3] = color_index_array_[k+index];
        }
        int maxColor;
        color_score.maxCoeff(&maxColor);
        sum_color_score[maxColor] += 1;
      }
    }
  }
  ROS_INFO("Interior Count: %i", count);
  ROS_INFO("Interior Sum: %f", sum_color_score.sum());
  int sumMaxColor;
  sum_color_score.maxCoeff(&sumMaxColor);
  return sumMaxColor;
}

