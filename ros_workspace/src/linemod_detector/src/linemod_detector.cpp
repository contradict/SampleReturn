#include <ros/ros.h>
#include <ros/console.h>
#include <stereo_msgs/DisparityImage.h>
#include <linemod_detector/NamedPoint.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// Function prototypes
void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                        CV_RGB(0, 255, 0),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m)
  {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}

class LineMOD_Detector
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber color_sub;
  ros::Subscriber depth_sub;
  ros::Subscriber disp_sub;
  ros::Subscriber cam_info_sub;
  ros::Publisher img_point_pub;
  ros::Publisher point_pub;
  std::vector<cv::Mat> sources;
  cv::Mat color_img;
  cv::Mat display;
  cv::Mat K;
  cv::Ptr<cv::linemod::Detector> detector;
  int matching_threshold;
  int num_modalities;
  int num_classes;
  bool got_color;
  float pub_threshold;
  float min_depth;
  float max_depth;
  float min_count;

  public:
  LineMOD_Detector(): it(nh)
  {
    color_sub = it.subscribe("color", 1, &LineMOD_Detector::colorCallback, this);
    depth_sub = nh.subscribe("depth", 1, &LineMOD_Detector::depthCallback, this);
    disp_sub = nh.subscribe("disparity", 1, &LineMOD_Detector::disparityCallback, this);
    cam_info_sub = nh.subscribe("cam_info", 1, &LineMOD_Detector::cameraInfoCallback, this);
    img_point_pub = nh.advertise<linemod_detector::NamedPoint>("img_point", 1);
    point_pub = nh.advertise<linemod_detector::NamedPoint>("point", 1);
    matching_threshold = 80;
    got_color = false;
    K = cv::Mat(3,3,CV_64FC1);

    std::string filename;
    ros::param::get("~template_file", filename);
    ros::param::get("~pub_threshold", LineMOD_Detector::pub_threshold);
    ros::param::get("~min_depth", LineMOD_Detector::min_depth);
    ros::param::get("~max_depth", LineMOD_Detector::max_depth);
    ros::param::get("~min_count", LineMOD_Detector::min_count);

    ROS_DEBUG("Pub Threshold:%f ", LineMOD_Detector::pub_threshold);

    // Initialize LINEMOD data structures
    detector = readLinemod(filename);
    num_modalities = (int)detector->getModalities().size();
    std::cout << num_modalities << std::endl;
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    for (int i = 0; (i < 3); i++)
    {
      for (int j = 0; (j < 3); j++)
      {
        LineMOD_Detector::K.at<double>(i,j) = msg.K.at(3*i+j);
      }
    }
    //std::cout << LineMOD_Detector::K << std::endl;
  }

  void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg)
  {
    ROS_DEBUG("Pub Threshold:%f ", LineMOD_Detector::pub_threshold);
    if (!LineMOD_Detector::got_color)
    {
      return;
    }
    bool show_match_result = true;
    cv_bridge::CvImagePtr disp_ptr;
    try
    {
      disp_ptr = cv_bridge::toCvCopy(msg->image, "");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("disp cv_bridge exception: %s", e.what());
      return;
    }

    float f = msg->f;
    float T = msg->T;
    cv::Mat depth_img = (f*T*1000)/(disp_ptr->image).clone();
    depth_img.convertTo(depth_img, CV_16U);
    LineMOD_Detector::sources.push_back(LineMOD_Detector::color_img);
    LineMOD_Detector::sources.push_back(depth_img);

      // Perform matching
      std::vector<cv::linemod::Match> matches;
      std::vector<std::string> class_ids;
      std::vector<cv::Mat> quantized_images;

      LineMOD_Detector::detector->match(sources, (float)LineMOD_Detector::matching_threshold, matches, class_ids, quantized_images);

      LineMOD_Detector::num_classes = detector->numClasses();
      ROS_DEBUG("Num Classes: %u", LineMOD_Detector::num_classes);
      int classes_visited = 0;
      std::set<std::string> visited;

      for (int i = 0; (i < (int)matches.size()) && (classes_visited < LineMOD_Detector::num_classes); ++i)
      {
        cv::linemod::Match m = matches[i];
        ROS_DEBUG("Matching count: %u", i);

        if (visited.insert(m.class_id).second)
        {
          ++classes_visited;

          if (show_match_result)
          {
            ROS_DEBUG("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
                   m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
            printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
                   m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
          }

          // Draw matching template
          const std::vector<cv::linemod::Template>& templates = LineMOD_Detector::detector->getTemplates(m.class_id, m.template_id);
          drawResponse(templates, LineMOD_Detector::num_modalities, LineMOD_Detector::display, cv::Point(m.x, m.y), LineMOD_Detector::detector->getT(0));
          if (m.similarity > LineMOD_Detector::pub_threshold)
          {
            LineMOD_Detector::publishPoint(templates, m, depth_img);
          }

        }
      }

      LineMOD_Detector::sources.clear();
  }


  void depthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!LineMOD_Detector::got_color)
    {
      return;
    }
    cv_bridge::CvImagePtr depth_ptr;
    bool show_match_result = true;
    try
    {
      depth_ptr = cv_bridge::toCvCopy(msg, "");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("depth cv_bridge exception: %s", e.what());
      return;
    }
    LineMOD_Detector::sources.push_back(LineMOD_Detector::color_img);
    LineMOD_Detector::sources.push_back(depth_ptr->image);


      // Perform matching
      std::vector<cv::linemod::Match> matches;
      std::vector<std::string> class_ids;
      std::vector<cv::Mat> quantized_images;

      LineMOD_Detector::detector->match(sources, (float)LineMOD_Detector::matching_threshold, matches, class_ids, quantized_images);

      LineMOD_Detector::num_classes = detector->numClasses();
      int classes_visited = 0;
      std::set<std::string> visited;

      for (int i = 0; (i < (int)matches.size()) && (classes_visited < LineMOD_Detector::num_classes); ++i)
      {
        cv::linemod::Match m = matches[i];

        if (visited.insert(m.class_id).second)
        {
          ++classes_visited;

          if (show_match_result)
          {
            printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
                   m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
          }

          // Draw matching template
          const std::vector<cv::linemod::Template>& templates = LineMOD_Detector::detector->getTemplates(m.class_id, m.template_id);
          drawResponse(templates, LineMOD_Detector::num_modalities, LineMOD_Detector::display, cv::Point(m.x, m.y), LineMOD_Detector::detector->getT(0));
          //std::cout << "pub_threshold " << LineMOD_Detector::pub_threshold << std::endl;
          if (m.similarity > LineMOD_Detector::pub_threshold)
          {
            LineMOD_Detector::publishPoint(templates, m, depth_ptr->image);
          }

        }
      }

      LineMOD_Detector::sources.clear();
  }

  void colorCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr color_ptr;
    try
    {
      color_ptr = cv_bridge::toCvCopy(msg, "");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("color cv_bridge exception: %s", e.what());
      return;
    }

    LineMOD_Detector::got_color = true;
    cv::Mat lab_img;
    cv::cvtColor(color_ptr->image,lab_img,CV_RGB2Lab);

    LineMOD_Detector::display = color_ptr->image.clone();
    LineMOD_Detector::color_img = lab_img;

  }

  void publishPoint(const std::vector<cv::linemod::Template>& templates, cv::linemod::Match m, cv::Mat& depth)
  {
    ROS_DEBUG("Publishing POint");
    linemod_detector::NamedPoint img_point_msg;
    linemod_detector::NamedPoint point_msg;
    img_point_msg.name = m.class_id;
    //img_point_msg.header =
    // We only care about the base pyramid level gradient modality
    img_point_msg.point.x = m.x + templates[1].width/2;
    img_point_msg.point.y = m.y + templates[1].height/2;
    ROS_DEBUG("Point x and y: %f, %f", img_point_msg.point.x, img_point_msg.point.y);

    LineMOD_Detector::img_point_pub.publish(img_point_msg);

    //Check for non-zero
    cv::Rect r(m.x,m.y,templates[1].width,templates[1].height);
    cv::Mat r_img = depth(r).clone();
    r_img.convertTo(r_img, CV_32F);
    //cv::imshow("r_img",r_img);
    char key = (char)cv::waitKey(10);
    int count = cv::countNonZero(r_img);
    if (count > LineMOD_Detector::min_count)
    {
      //Drop too close or too far
      cv::threshold(r_img,r_img,LineMOD_Detector::min_depth, 0, cv::THRESH_TOZERO);
      cv::threshold(r_img,r_img,LineMOD_Detector::max_depth, 0, cv::THRESH_TOZERO_INV);
      //Grab remaining extrema, compute mean
      //The farpoints will probably be ground, the close will be the near side of the object
      r_img = r_img.reshape(0,1);
      cv::sort(r_img,r_img,CV_SORT_DESCENDING);
      double sum = cv::sum(r_img.colRange(0,10))(0) + cv::sum(r_img.colRange(count-10,count))(0);
      double ave = sum/20;
      //The average is the average depth in mm
      //Now we reproject the center point
      //std::cout << ave << std::endl;
    }
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_listener");
  LineMOD_Detector ld;
  ros::spin();
}
