// Timing test for BMS saliency algorithm, which seems to be too slow for
// all three Basler cameras at a reasonable (~1000px wide) size and good
// parameters.

#include "BMS.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>

int main( int argc, char** argv)
{
  // Initialize BMS object
  bool norm = true;
  bool handle_border = false;
  int width = atoi(argv[1]);
  int sample_step = atoi(argv[2]);
  int dw1 = atoi(argv[3]);
  int ow = atoi(argv[4]);

  BMS bms = BMS(dw1, ow, norm, handle_border);

  // Load an image
  std::string path = "/home/zlizer/src/SampleReturn/ros_workspace/";
  cv::Mat img = cv::imread(argv[5]);
  cv::cvtColor(img,img,CV_BGR2RGB);

  // Do whatever resizing we're going to do
  int height = width * (float(img.rows)/img.cols);
  std::cout << "Height: " << height << " Width: " << width << std::endl;
  cv::Mat small_nearest, small_area, bms_out_sab_n, bms_out_vab_n, bms_out_sab_area, bms_out_vab_area;

  cv::resize(img, small_nearest, cv::Size(width,height), 0.0, 0.0, cv::INTER_NEAREST);
  cv::resize(img, small_area, cv::Size(width,height), 0.0, 0.0, cv::INTER_AREA);

  // Compute Saliency, with timing
  bms.computeSaliency(small_nearest,sample_step,0);
  bms_out_vab_n = bms.getSaliencyMap().clone();

  bms.computeSaliency(small_nearest,sample_step,1);
  bms_out_sab_n = bms.getSaliencyMap().clone();

  bms.computeSaliency(small_area,sample_step,0);
  bms_out_vab_area = bms.getSaliencyMap().clone();

  bms.computeSaliency(small_area,sample_step,1);
  bms_out_sab_area = bms.getSaliencyMap().clone();

  cv::cvtColor(img,img,CV_RGB2BGR);

  cv::namedWindow("BMS Area Vab",WINDOW_NORMAL);
  cv::namedWindow("BMS Area Sab",WINDOW_NORMAL);
  cv::namedWindow("BMS Nearest Vab",WINDOW_NORMAL);
  cv::namedWindow("BMS Nearest Sab",WINDOW_NORMAL);
  cv::namedWindow("Image",WINDOW_NORMAL);
  cv::namedWindow("Small Area",WINDOW_NORMAL);
  cv::namedWindow("Small Nearest",WINDOW_NORMAL);
  cv::imshow("BMS Area Vab",bms_out_vab_area);
  cv::imshow("BMS Area Sab",bms_out_sab_area);
  cv::imshow("BMS Nearest Vab",bms_out_vab_n);
  cv::imshow("BMS Nearest Sab",bms_out_sab_n);
  cv::imshow("Image",img);
  cv::imshow("Small Area",small_area);
  cv::imshow("Small Nearest",small_nearest);
  cv::resizeWindow("BMS Area Vab", 1000, 500);
  cv::resizeWindow("BMS Area Sab", 1000, 500);
  cv::resizeWindow("BMS Nearest Vab", 1000, 500);
  cv::resizeWindow("BMS Nearest Sab", 1000, 500);
  cv::resizeWindow("Image", 1000, 1000);
  cv::moveWindow("BMS Area Vab", 2000, 0);
  cv::moveWindow("BMS Area Sab", 3000, 0);
  cv::moveWindow("BMS Nearest Vab", 2000, 500);
  cv::moveWindow("BMS Nearest Sab", 3000, 500);
  for(;;) {
    int keycode = waitKey(0);
    if( keycode == 27 )
        break;
  }
}
