#include <ros/ros.h>
#include <ros/console.h>
#include <stereo_msgs/DisparityImage.h>
#include <samplereturn_msgs/NamedPoint.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <color_naming.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_listener.h>
#include <samplereturn_msgs/Enable.h>

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
  ros::Subscriber disparity_sub;
  ros::Subscriber left_cam_info_sub;
  ros::Subscriber right_cam_info_sub;
  ros::Publisher img_point_pub;
  ros::Publisher point_pub;
  ros::Publisher debug_img_pub;
  std::vector<cv::Mat> sources;
  cv::Mat color_img;
  cv::Mat disparity_img;
  cv::Mat display;
  cv::Mat K;
  cv::Ptr<cv::linemod::Detector> detector;
  sensor_msgs::CameraInfo right_camera_info_msg;
  int matching_threshold;
  int num_modalities;
  int num_classes;
  bool got_color;
  bool got_right_camera_info_;
  bool got_disp_;
  float pub_threshold;
  float min_depth;
  float max_depth;
  float min_count;
  bool _publish_debug_img;

  ColorNaming cn;
  //image_geometry::PinholeCameraModel cam_model_;
  image_geometry::StereoCameraModel cam_model_;

  tf::TransformListener listener_;
  ros::ServiceServer service_;
  bool enabled_;

  public:
  LineMOD_Detector(): it(nh)
  {
    color_sub = it.subscribe("color", 1, &LineMOD_Detector::colorCallback, this);
    left_cam_info_sub = nh.subscribe("left_cam_info", 1, &LineMOD_Detector::leftCameraInfoCallback, this);
    right_cam_info_sub = nh.subscribe("right_cam_info", 1, &LineMOD_Detector::rightCameraInfoCallback, this);
    disparity_sub = nh.subscribe("disparity", 1, &LineMOD_Detector::disparityCallback, this);
    img_point_pub = nh.advertise<samplereturn_msgs::NamedPoint>("img_point", 1);
    debug_img_pub = nh.advertise<sensor_msgs::Image>("linemod_2d_debug_img", 1);
    point_pub = nh.advertise<samplereturn_msgs::NamedPoint>("point", 1);
    matching_threshold = 80;
    got_color = false;
    K = cv::Mat(3,3,CV_64FC1);

    service_ = nh.advertiseService("enable",&LineMOD_Detector::enable,this);

    std::string filename;
    ros::param::get("~template_file", filename);
    ros::param::get("~pub_threshold", LineMOD_Detector::pub_threshold);
    ros::param::get("~min_depth", LineMOD_Detector::min_depth);
    ros::param::get("~max_depth", LineMOD_Detector::max_depth);
    ros::param::get("~min_count", LineMOD_Detector::min_count);
    ros::param::param<bool>("~publish_debug_img", _publish_debug_img, true);

    ROS_DEBUG("Pub Threshold:%f ", LineMOD_Detector::pub_threshold);

    // Initialize LINEMOD data structures
    detector = readLinemod(filename);
    num_modalities = (int)detector->getModalities().size();
    std::cout << num_modalities << std::endl;

    enabled_ = true;
    got_right_camera_info_ = false;
    got_disp_ = false;
  }

  bool enable(samplereturn_msgs::Enable::Request &req,
              samplereturn_msgs::Enable::Response &res) {
    enabled_ = req.state;
    res.state = enabled_;
    return true;
  }

  void leftCameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    if (!got_right_camera_info_) {
      return;
    }
    right_camera_info_msg.header.frame_id = msg.header.frame_id;
    cam_model_.fromCameraInfo(msg, right_camera_info_msg);
    for (int i = 0; (i < 3); i++)
    {
      for (int j = 0; (j < 3); j++)
      {
        LineMOD_Detector::K.at<double>(i,j) = msg.K.at(3*i+j);
      }
    }
    //std::cout << LineMOD_Detector::K << std::endl;
  }

  void rightCameraInfoCallback(const sensor_msgs::CameraInfo& msg)
  {
    right_camera_info_msg = msg;
    got_right_camera_info_ = true;
  }

  void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg)
  {
    if (!enabled_) {
      return;
    }
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

    //float f = msg->f;
    //float T = msg->T;
    //cv::Mat depth_img = (f*T*1000)/(disp_ptr->image).clone();
    //depth_img.convertTo(depth_img, CV_16U);
    disparity_img = (disp_ptr->image).clone();
    got_disp_ = true;
  }

  void colorCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!enabled_) {
      return;
    }
    if (!got_disp_) {
      return;
    }
    bool show_match_result = true;
    cv_bridge::CvImagePtr color_ptr;
    try
    {
      color_ptr = cv_bridge::toCvCopy(msg, "rgb8");
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

    cv::Mat blur;
    cv::medianBlur(lab_img, blur, 13);

    //LineMOD_Detector::sources.push_back(LineMOD_Detector::color_img);
    LineMOD_Detector::sources.push_back(blur);

    // Perform matching
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;

    LineMOD_Detector::detector->match(sources, (float)LineMOD_Detector::matching_threshold, matches, class_ids, quantized_images);

    LineMOD_Detector::num_classes = detector->numClasses();
    ROS_DEBUG("Num Classes: %u", LineMOD_Detector::num_classes);
    int classes_visited = 0;
    std::set<std::string> visited;

    ROS_DEBUG("Matches size: %u", (int)matches.size());
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

        cv::Mat mask = cv::Mat::zeros(display.size(), CV_8U);
        std::vector<cv::Point> hull;
        hull = templateConvexHull(templates, LineMOD_Detector::num_modalities, cv::Point(m.x,m.y), mask.size(),
            mask);
        //cv::imshow("mask", mask);
        //std::cout << "Mask size: " << mask.size() << std::endl;
        //std::cout << "Mask channels: " << mask.channels() << std::endl;
        //cv::imshow("display", LineMOD_Detector::display);
        //cv::waitKey(10);
        Eigen::Matrix<float,11,1> interiorColor(Eigen::Matrix<float,11,1>::Zero());
        //interiorColor = cn.computeInteriorColor(LineMOD_Detector::display, mask);
        interiorColor = cn.computeInteriorColorStats(LineMOD_Detector::display, mask);
        //std::cout << "Interior color: " << interiorColor << std::endl;
        std::string dominant_color = cn.getDominantColor(interiorColor);
        std::cout << "Dominant color " << dominant_color << std::endl;

        cv::RotatedRect rect = cv::minAreaRect(hull);
        float angle;
        if (rect.size.width > rect.size.height) {
          angle = rect.angle + 90;
        }
        else {
          angle = rect.angle + 180;
        }
        angle -= 90.;
        angle = angle*(M_PI/180);

        std::cout << "Angle: " << angle << std::endl;

        if (dominant_color != "green") {
          drawResponse(templates, LineMOD_Detector::num_modalities, LineMOD_Detector::display, cv::Point(m.x, m.y), LineMOD_Detector::detector->getT(0));
        }

        if (m.similarity > LineMOD_Detector::pub_threshold && dominant_color!="green")
        {
          if (m.class_id.c_str() == "red_puck" &&
              (dominant_color=="red" || dominant_color=="pink" || dominant_color=="purple" || dominant_color=="orange"))
          {
            LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                angle, samplereturn_msgs::NamedPoint::RED_PUCK);
          }
          if (m.class_id.c_str() == "orange_pipe" &&
              (dominant_color=="orange" || dominant_color=="white" || dominant_color=="yellow"))
          {
            LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                angle, samplereturn_msgs::NamedPoint::ORANGE_PIPE);
          }
          if (m.class_id.c_str() == "pre_cached" &&
              (dominant_color=="white"))
          {
            LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                angle, samplereturn_msgs::NamedPoint::PRE_CACHED);
          }
          if (m.class_id.c_str() == "wood_cube" &&
              (dominant_color=="yellow" || dominant_color=="brown"))
          {
            LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                angle, samplereturn_msgs::NamedPoint::WOODEN_CUBE);
          }
          if (m.class_id.c_str() == "pink_tennis_ball" &&
              (dominant_color=="pink" || dominant_color=="white"))
          {
            LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                angle, samplereturn_msgs::NamedPoint::PINK_TENNIS_BALL);
          }
          if (m.class_id.c_str() == "colored_ball" &&
              dominant_color!="brown" && dominant_color!="white" && dominant_color!="grey")
          {
            LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                angle, samplereturn_msgs::NamedPoint::COLORED_BALL);
          }
        }

      }
    }

   if(_publish_debug_img)
   {
       sensor_msgs::ImagePtr debugmsg = cv_bridge::CvImage(color_ptr->header, color_ptr->encoding, display).toImageMsg();
       debug_img_pub.publish(debugmsg);
   }

    LineMOD_Detector::sources.clear();
  }

//  void publishPoint(const std::vector<cv::linemod::Template>& templates, cv::linemod::Match m,
//      std_msgs::Header header)
//  {
//    ROS_DEBUG("Publishing Point");
//    samplereturn_msgs::NamedPoint point_msg;
//

  void publishPoint(const std::vector<cv::linemod::Template>& templates, cv::linemod::Match m,
      std_msgs::Header header, float grip_angle, int sample_id)
  {
    ROS_DEBUG("Publishing Img Point");
    samplereturn_msgs::NamedPoint img_point_msg;
    img_point_msg.name = m.class_id;
    img_point_msg.sample_id = sample_id;
    img_point_msg.header = header;
    // We only care about the base pyramid level gradient modality
    img_point_msg.point.x = m.x + templates[1].width/2;
    img_point_msg.point.y = m.y + templates[1].height/2;
    img_point_msg.grip_angle = grip_angle;
    ROS_DEBUG("Point x and y: %f, %f", img_point_msg.point.x, img_point_msg.point.y);

    if (cam_model_.initialized()) {
      samplereturn_msgs::NamedPoint point_msg;
      geometry_msgs::PointStamped temp_point;
      geometry_msgs::PointStamped odom_point;
      temp_point.header = header;
      cv::Point3d xyz;
      float median_disp;
      float half_width = templates[1].width/2;
      float half_height = templates[1].height/2;

      cv::Mat sub_disp = disparity_img(cv::Range(m.y,m.y+2*half_height),
          cv::Range(m.x,m.x+2*half_width)).clone();
      cv::Mat flat = sub_disp.reshape(0,1);
      cv::sort(flat, flat, CV_SORT_ASCENDING+CV_SORT_EVERY_ROW);
      median_disp = flat.at<float>(0,(flat.cols/2));
      std::cout << "Median Disp: " << median_disp << std::endl;

      cam_model_.projectDisparityTo3d(cv::Point2d(m.x+templates[1].width/2,m.y+templates[1].height/2),
          median_disp, xyz);
      temp_point.point.x = xyz.x;
      temp_point.point.y = xyz.y;
      temp_point.point.z = xyz.z;
      bool wait =
        listener_.waitForTransform("/odom", header.frame_id, header.stamp, ros::Duration(0.03));
      if (!wait) {
        return;
      }
      listener_.transformPoint("/odom", temp_point, odom_point);

      //std::cout << "Camera 3D point: " << temp_point << std::endl;
      point_msg.name = m.class_id;
      point_msg.sample_id = sample_id;
      point_msg.header = header;
      point_msg.header.frame_id = "/odom";
      point_msg.grip_angle = grip_angle;
      point_msg.point = odom_point.point;
      LineMOD_Detector::point_pub.publish(point_msg);
    }

    LineMOD_Detector::img_point_pub.publish(img_point_msg);
  }

  std::vector<cv::Point> templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                          int num_modalities, cv::Point offset, cv::Size size,
                          cv::Mat& dst)
  {
    std::vector<cv::Point> points;
    for (int m = 0; m < num_modalities; ++m)
    {
      for (int i = 0; i < (int)templates[m].features.size(); ++i)
      {
        cv::linemod::Feature f = templates[m].features[i];
        points.push_back(cv::Point(f.x, f.y) + offset);
      }
    }

    std::vector<cv::Point> hull;
    cv::convexHull(points, hull);

    //dst = cv::Mat::zeros(size, CV_8U);
    const int hull_count = (int)hull.size();
    const cv::Point* hull_pts = &hull[0];
    cv::fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
    return hull;
  }

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "Line2D_Detector");
  LineMOD_Detector ld;
  ros::spin();
}
