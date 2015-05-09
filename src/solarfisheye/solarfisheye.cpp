/**
 * @function solarfisheye.cpp
 * @brief Example code to find sun in fisheye image
 * @author James Sarrett
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 10;
int max_thresh = 255;

/// Function header
void thresh_callback(int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );

  /// Convert image to gray
  cvtColor( src, src_gray, COLOR_BGR2GRAY );

  /// Create Window
  const char* source_window = "Source";
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, src );

  createTrackbar( "Threshold:", "Source", &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );

  waitKey(0);
  return(0);
}

/**
 * @function thresh_callback
 */
void thresh_callback(int, void* )
{
  Mat src_thr = Mat::zeros( src.size(), CV_8UC1 );
  threshold(src_gray, src_thr, thresh, 255, cv::THRESH_TOZERO);

  /// Get the moments
  Moments mu = cv::moments(src_thr, false );
  ///  Get the mass center:
  Point2f mc = Point2f( static_cast<float>(mu.m10/mu.m00) , static_cast<float>(mu.m01/mu.m00) );

  Mat drawing = Mat::zeros( src_thr.size(), CV_8UC3 );
  cvtColor( src_thr, drawing, COLOR_GRAY2BGR );
  circle( drawing, mc, 20, Scalar(0, 255, 0), 1, 8, 0 );

  /// Show in a window
  namedWindow( "Threshold", WINDOW_AUTOSIZE );
  imshow( "Threshold", drawing);
}
