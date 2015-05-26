/**
 * @function solarfisheye.cpp
 * @brief Example code to find sun in fisheye image
 * @author James Sarrett
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
Mat cameraMatrix, distCoeffs;
int thresh = 10;
int max_thresh = 255;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// taken from cv::fisheye::undistortPoints

void unprojectPointsFisheye( InputArray distorted, OutputArray undistorted, InputArray K, InputArray D, InputArray R, InputArray P)
{
    // will support only 2-channel data now for points
    CV_Assert(distorted.type() == CV_32FC2 || distorted.type() == CV_64FC2);
    undistorted.create(distorted.size(), CV_MAKETYPE(distorted.depth(), 3));

    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(D.total() == 4 && K.size() == Size(3, 3) && (K.depth() == CV_32F || K.depth() == CV_64F));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        Matx33f camMat = K.getMat();
        f = Vec2f(camMat(0, 0), camMat(1, 1));
        c = Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        Matx33d camMat = K.getMat();
        f = Vec2d(camMat(0, 0), camMat(1, 1));
        c = Vec2d(camMat(0, 2), camMat(1, 2));
    }

    Vec4d k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

    cv::Matx33d RR = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    if(!P.empty())
    {
        cv::Matx33d PP;
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
        RR = PP * RR;
    }

    // start undistorting
    const cv::Vec2f* srcf = distorted.getMat().ptr<cv::Vec2f>();
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec3f* dstf = undistorted.getMat().ptr<cv::Vec3f>();
    cv::Vec3d* dstd = undistorted.getMat().ptr<cv::Vec3d>();

    size_t n = distorted.total();
    int sdepth = distorted.depth();

    for(size_t i = 0; i < n; i++ )
    {
        Vec2d pi = sdepth == CV_32F ? (Vec2d)srcf[i] : srcd[i];  // image point
        Vec2d pw((pi[0] - c[0])/f[0], (pi[1] - c[1])/f[1]);      // world point

        double scale = 1.0;

        double theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);
        if (theta_d > 1e-8)
        {
            // compensate distortion iteratively
            double theta = theta_d;
            for(int j = 0; j < 10; j++ )
            {
                double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
                theta = theta_d / (1 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
            }

            //scale = std::tan(theta) / theta_d;
            scale = theta / theta_d;
        }

        Vec2d pu = pw * scale; //undistorted point

        // reproject
        Vec3d pr = RR * Vec3d(pu[0], pu[1], 1.0); // rotated point optionally multiplied by new camera matrix
        //Vec2d fi(pr[0]/pr[2], pr[1]/pr[2]);       // final
        Vec3d fi;       // final
        normalize(pr, fi);

        if( sdepth == CV_32F )
            dstf[i] = fi;
        else
            dstd[i] = fi;
    }
}

/// Function header
void thresh_callback(int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load source image and convert it to gray
  src = imread(argv[2], 1);

  /// Convert image to gray
  cvtColor( src, src_gray, COLOR_BGR2GRAY );

  /// Load calibration parameters
  FileStorage fs(argv[1], FileStorage::READ);
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;

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

  vector<Point2f> pts;
  pts.push_back(mc);
  vector<Point3f> undist_pts;
  unprojectPointsFisheye(pts, undist_pts, cameraMatrix, distCoeffs, Mat(), Mat());
  cout << pts << endl;
  cout << undist_pts << endl;
  printf("intensity: %i\n", src_thr.at<uint8_t>(mc));
  printf("[[%lf\t%lf]\n[%lf\t%lf]]\n", mu.mu20, mu.mu11, mu.mu11, mu.mu02);
  printf("%lf - <%lf,%lf>\n", mu.m00, sqrt(mu.mu20), sqrt(mu.mu02));
  //cout << src_thr.at<uint8_t>(mc) << endl;

  /// Show in a window
  namedWindow( "Threshold", WINDOW_AUTOSIZE );
  imshow( "Threshold", drawing);
}
