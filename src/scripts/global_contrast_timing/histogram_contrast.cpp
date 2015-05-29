#include <math.h>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  // Load no-sample image or images, trim sky, compute RGB hist
  // From metal_sample: frame 23, 25, 30, 36, 40, 47, 48
  cv::Mat img;
  img = cv::imread("/home/zlizer/src/SampleReturn/ros_workspace/park_dslr/frame0025.jpg");
  cv::cvtColor(img,img,cv::COLOR_BGR2RGB);
  cv::Mat small;
  cv::Mat lab_small, hsv_small;
  cv::resize(img.rowRange(600,img.rows),small,cv::Size(1000.,(img.rows-600)*(1000./img.cols)),
      0.0,0.0,cv::INTER_AREA);
  cv::cvtColor(small,lab_small,cv::COLOR_RGB2Lab);
  cv::cvtColor(small,hsv_small,cv::COLOR_RGB2HSV);

  int channels[] = {0,1,2};
  int hist_size[] = {12,12,12};
  float r_range[] = {0,256};
  float g_range[] = {0,256};
  float b_range[] = {0,256};
  const float* ranges[] = {r_range,g_range,b_range};

  std::chrono::time_point<std::chrono::system_clock> start_t, end_t;

  start_t = std::chrono::system_clock::now();
  cv::Mat hist;
  cv::calcHist(&small, 1, channels, cv::Mat(), hist, 3, hist_size, ranges);

  float sum = 0;
  struct index_score {
    float count = 0;
    uchar R = 0;
    uchar G = 0;
    uchar B = 0;
    uchar L = 0;
    uchar a = 0;
    uchar b = 0;
    uchar H = 0;
    uchar S = 0;
    uchar V = 0;
  } __attribute__((packed));
  std::vector<index_score> top_indices;
  // Normalize hist
  for (int i=0; i<hist_size[0]; i++) {
    for (int j=0; j<hist_size[1]; j++) {
      for (int k=0; k<hist_size[2]; k++) {
        sum += hist.at<float>(i,j,k);
        float bin_width = r_range[1]/float(hist_size[0]);
        struct index_score i_s;
        i_s.count = hist.at<float>(i,j,k);
        i_s.R = (i+0.5)*bin_width;
        i_s.G = (j+0.5)*bin_width;
        i_s.B = (k*0.5)*bin_width;
        // Add Lab space for distance metric
        cv::Mat rgb(1,1,CV_8UC3,&i_s.R);
        cv::Mat Lab(1,1,CV_8UC3,&i_s.L);
        cv::Mat HSV(1,1,CV_8UC3,&i_s.H);
        cv::cvtColor(rgb,Lab,CV_RGB2Lab);
        cv::cvtColor(rgb,HSV,CV_RGB2HSV);
        top_indices.push_back(i_s);
      }
    }
  }

  // Reduce hist to n ~= 100 bins (try to cover some fraction of pixels. e.g.95%)
  std::sort(top_indices.begin(), top_indices.end(), [](struct index_score a, struct index_score b) {
      return b.count < a.count;
      });

  end_t = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end_t-start_t;
  std::cout << "Hist elapsed: " << elapsed.count() << std::endl;

  start_t = std::chrono::system_clock::now();
  // For query image, iterate over hist bins to compute per-pixel score,
  // which is distance between query pixel and hist bin color * hist bin count
  cv::Mat out = cv::Mat::zeros(small.rows,small.cols,CV_32FC1);
  for (int n=0; n<40; n++) {
    for (int i=0; i<small.rows;i++) {
      for (int j=0; j<small.cols;j++) {
        cv::Vec3b lab_px = lab_small.at<cv::Vec3b>(i,j);
        cv::Vec3b hsv_px = hsv_small.at<cv::Vec3b>(i,j);
        float diff = abs(hsv_px[0] - top_indices[n].H)
          + abs(lab_px[1] - top_indices[n].a) + abs(lab_px[2] - top_indices[n].b);
        out.at<double>(i,j) += top_indices[n].count*diff/sum;
      }
    }
  }
  double minVal, maxVal;
  cv::minMaxLoc(out,&minVal,&maxVal);
  std::cout << out.at<float>(0,0) << std::endl;
  std::cout << out.at<float>(1,1) << std::endl;
  std::cout << "Min Val out: " << minVal << std::endl;
  std::cout << "Max Val out: " << maxVal << std::endl;

  end_t = std::chrono::system_clock::now();
  elapsed = end_t-start_t;
  std::cout << "Image elapsed: " << elapsed.count() << std::endl;

  cv::Mat thresh;
  cv::threshold(out,thresh,110,1.0,cv::THRESH_BINARY);
  cv::namedWindow("img",CV_WINDOW_NORMAL);
  cv::imshow("img",out/255.);
  cv::waitKey(0);


  return 0;
}
