#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  //Load color and disparity images
  cv::Mat filtered_disparity;
  cv::Mat color = cv::imread("color.jpg");
  cv::Mat disparity = cv::imread("disp.jpg", CV_LOAD_IMAGE_GRAYSCALE);
  //Construct GPU things
  cv::gpu::GpuMat d_color, d_disparity, d_filtered_disparity;
  //cv::gpu::DisparityBilateralFilter DBF(96, 3, 7);
  //std::cout << std::atoi(argv[1]) << std::endl;
  cv::gpu::DisparityBilateralFilter DBF(std::atoi(argv[1]), std::atoi(argv[2]), std::atoi(argv[3]));
  d_color.upload(color);
  d_disparity.upload(disparity);
  //Compute
  DBF(d_disparity, d_color, d_filtered_disparity);
  d_filtered_disparity.download(filtered_disparity);
  //Display
  std::cout << "Filtered Disparity shape: " << filtered_disparity.rows <<
    "," << filtered_disparity.cols << std::endl;
  //cv::namedWindow("filtered_disparity");
  cv::imshow("filtered_disparity", filtered_disparity);
  cv::imshow("raw_disparity", disparity);
  cv::waitKey();
}
