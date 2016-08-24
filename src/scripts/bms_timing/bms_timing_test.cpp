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
  //cv::Mat img = cv::imread(path+"Basler_outdoor_exp5000_tree_pre_red.png");
  cv::cvtColor(img,img,CV_BGR2RGB);
  // Do whatever resizing we're going to do
  int height = width * (float(img.rows)/img.cols);
  std::cout << "Height: " << height << " Width: " << width << std::endl;
  cv::Mat small, bms_out;
  std::chrono::time_point<std::chrono::system_clock> start, resize_end,
    comp_sal_end, get_sal_end;
  std::chrono::duration<double> elapsed_resize, elapsed_comp_sal, elapsed_get_sal;

  start = std::chrono::system_clock::now();
  cv::resize(img, small, cv::Size(width,height), 0.0, 0.0, cv::INTER_AREA);
  resize_end = std::chrono::system_clock::now();

  // Compute Saliency, with timing
  bms.computeSaliency(small,sample_step);
  comp_sal_end = std::chrono::system_clock::now();
  bms_out = bms.getSaliencyMap().clone();
  get_sal_end = std::chrono::system_clock::now();

  elapsed_resize = resize_end-start;
  std::cout << "elapsed time resize: " << elapsed_resize.count() << std::endl;
  elapsed_comp_sal = comp_sal_end-resize_end;
  std::cout << "elapsed time comp sal: " << elapsed_comp_sal.count() << std::endl;
  elapsed_resize = get_sal_end-comp_sal_end;
  std::cout << "elapsed time get sal: " << elapsed_get_sal.count() << std::endl;

  cv::Mat bms_thresh;
  cv::threshold(bms_out, bms_thresh, atoi(argv[6]), 255, cv::THRESH_BINARY);

  cv::cvtColor(img,img,CV_RGB2BGR);

  cv::imwrite("small_image.png", small);
  cv::imwrite("small_bms.png", bms_out);

  cv::namedWindow("BMS",WINDOW_NORMAL);
  cv::namedWindow("BMS Thresh",WINDOW_NORMAL);
  //cv::moveWindow("BMS",1940,0);
  cv::imshow("BMS",bms_out);
  cv::imshow("BMS Thresh",bms_thresh);
  cv::namedWindow("Image",WINDOW_NORMAL);
  //cv::moveWindow("Image",2900,0);
  cv::imshow("Image",img);
  for(;;) {
    int keycode = waitKey(0);
    if( keycode == 27 )
        break;
  }
}
