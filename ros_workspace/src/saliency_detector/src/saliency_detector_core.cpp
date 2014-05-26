#include "saliency_detector_core.h"

//Constructor
SaliencyDetector::SaliencyDetector()
{
  //// Construct ColorName
  //std::string line;
  //std::ifstream infile(color_name_file);
  //std::vec color_vec;
  ////For the 4096 color version, 4096*11=45056
  ////For the full version, 32768*11=360448
  //double array[360448];
  //while (std::getline(infile, line)) {
  //  std::istringstream iss(line);
  //  while (iss >> value) {}
  //}
}

//Destructor
SaliencyDetector::~SaliencyDetector()
{
}

/* This is where the meat of the detector lies:
 * Pre-process the image
 * Compute Saliency map
 * Extract blobs
 * Post-process blobs
 * Return scored object detections
 */
void SaliencyDetector::messageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("messageCallback");
  cv_bridge::CvImagePtr cv_ptr;
  try {
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr = cv_bridge::toCvCopy(msg, "");
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  cv::Mat small;
  cv::resize(cv_ptr->image,small,cv::Size(600.0,cv_ptr->image.rows*(600.0/cv_ptr->image.cols)),0.0,0.0,cv::INTER_AREA);

  //bms_.computeSaliency(cv_ptr->image, bms_sample_step_);
  bms_.computeSaliency(small, bms_sample_step_);
  debug_bms_img_ = bms_.getSaliencyMap();

  ROS_INFO("messageCallback ended");
}

/* Dynamic reconfigure callback */
void SaliencyDetector::configCallback(saliency_detector::saliency_detector_paramsConfig &config, uint32_t level)
{
  // Construct BMS
  bms_ = BMS(config.bms_dilation_width_1, config.bms_opening_width,
      config.bms_normalize, config.bms_handle_border);

  bms_sample_step_ = config.bms_sample_step;
  bms_blur_std_ = config.bms_blur_std;

  // Construct BlobDetector/MSER
  //BlobParams_ cv::SimpleBlobDetector::Params();
  //BlobParams_.filterByColor = config.filterByColor;
  //BlobParams_.blobColor = config.blobColor;
  //BlobParams_.maxThreshold = config.maxThreshold;
  //BlobParams_.minThreshold = config.minThreshold;
  //Blob_ cv::SimpleBlobDetector(BlobParams_);
}

void SaliencyDetector::publishMessage(ros::Publisher *pub_message)
{
  std_msgs::Header header;
  sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"",debug_bms_img_).toImageMsg();
  pub_message->publish(debug_img_msg);
}
