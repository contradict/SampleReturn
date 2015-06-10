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
#include <BMS.h>
#include <linemod_detector/LinemodConfig.h>
#include <dynamic_reconfigure/server.h>
#include "new_modalities.hpp"

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

static cv::Ptr<cv::linemod::Detector> readInnerLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = cv::linemod::getInnerLINE();

  cv::FileStorage fs(filename, cv::FileStorage::READ);
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
  ros::Publisher pub_bms_img;
  dynamic_reconfigure::Server<linemod_detector::LinemodConfig> reconfigure;
  std::vector<cv::Mat> sources;
  cv::Mat color_img;
  cv::Mat disparity_img;
  float min_disp;
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
  bool hard_samples;
  bool _publish_debug_img;

  std::vector<std::string> interior_colors_vec_, exterior_colors_vec_;

  linemod_detector::LinemodConfig _config;

  ColorNaming cn;
  //image_geometry::PinholeCameraModel cam_model_;
  image_geometry::StereoCameraModel cam_model_;

  tf::TransformListener listener_;
  ros::ServiceServer service_;
  bool enabled_;

  std::string _detection_frame_id;

  public:
  LineMOD_Detector(): it(nh)
  {
    color_sub = it.subscribe("color", 1, &LineMOD_Detector::colorCallback, this);
    left_cam_info_sub = nh.subscribe("left_cam_info", 1, &LineMOD_Detector::leftCameraInfoCallback, this);
    right_cam_info_sub = nh.subscribe("right_cam_info", 1, &LineMOD_Detector::rightCameraInfoCallback, this);
    disparity_sub = nh.subscribe("disparity", 1, &LineMOD_Detector::disparityCallback, this);
    img_point_pub = nh.advertise<samplereturn_msgs::NamedPoint>("img_point", 1);
    debug_img_pub = nh.advertise<sensor_msgs::Image>("linemod_2d_debug_img", 1);
    pub_bms_img = nh.advertise<sensor_msgs::Image>("bms_debug", 1);
    point_pub = nh.advertise<samplereturn_msgs::NamedPoint>("point", 1);
    matching_threshold = 80;
    got_color = false;
    K = cv::Mat(3,3,CV_64FC1);

    ros::NodeHandle pnh("~");
    service_ = pnh.advertiseService("enable",&LineMOD_Detector::enable,this);

    std::string filename;
    ros::param::get("~template_file", filename);
    ros::param::get("~pub_threshold", LineMOD_Detector::pub_threshold);
    ros::param::get("~min_depth", LineMOD_Detector::min_depth);
    ros::param::get("~max_depth", LineMOD_Detector::max_depth);
    ros::param::get("~min_count", LineMOD_Detector::min_count);
    ros::param::get("~hard_samples", LineMOD_Detector::hard_samples);
    ros::param::param<bool>("~publish_debug_img", _publish_debug_img, true);
    ros::param::param<std::string>("~detection_frame_id", _detection_frame_id, "odom");

    ros::param::get("~interior_colors", interior_colors_vec_);
    ros::param::get("~exterior_colors", exterior_colors_vec_);

    reconfigure.setCallback(boost::bind(&LineMOD_Detector::configCallback, this,  _1, _2));

    ROS_DEBUG("Pub Threshold:%f ", LineMOD_Detector::pub_threshold);

    // Initialize LINEMOD data structures
    if (!hard_samples) {
      detector = readLinemod(filename);
      num_modalities = (int)detector->getModalities().size();
    }
    else {
      detector = readInnerLinemod(filename);
      num_modalities = (int)detector->getModalities().size();
    }
    std::cout << num_modalities << std::endl;

    enabled_ = false;
    got_right_camera_info_ = false;
    got_disp_ = false;
  }

  void configCallback(linemod_detector::LinemodConfig &config, uint32_t level)
  {
      _config = config;
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
    min_disp = msg->min_disparity;
    got_disp_ = true;
  }

  double unwrap90(double angle)
  {
      while(angle>M_PI/2)
          angle -= M_PI;
      while(angle<-M_PI/2)
          angle += M_PI;
      return angle;
  }

  float computeGripAngle(const std::vector<cv::Point>& hull)
  {
      float angle;
      cv::RotatedRect rect = cv::minAreaRect(hull);
      ROS_INFO("Meausred angle: %f width: %f height: %f",
              rect.angle, rect.size.width, rect.size.height);
      if(rect.size.width>rect.size.height)
      {
          angle = -rect.angle*M_PI/180+M_PI_2;
      }
      else
      {
          angle = -rect.angle*M_PI/180;
      }
      angle = unwrap90(angle);

      ROS_INFO("Angle: %f", angle);
      return angle;
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
      //cv::Mat lab_img;
      //cv::cvtColor(color_ptr->image,lab_img,CV_RGB2Lab);

      cv::Mat blur;
      cv::medianBlur(color_ptr->image, blur, 13);

      LineMOD_Detector::display = color_ptr->image.clone();
      //LineMOD_Detector::display = blur.clone();
      //LineMOD_Detector::color_img = lab_img;
      //LineMOD_Detector::color_img = color_ptr->image.clone();
      if (hard_samples) {
          LineMOD_Detector::color_img = color_ptr->image.clone();
      }
      else {
          LineMOD_Detector::color_img = blur.clone();
      }

      LineMOD_Detector::sources.push_back(LineMOD_Detector::color_img);
      //LineMOD_Detector::sources.push_back(blur);

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
      float best_match_similarity = 0;
      int best_match_idx = -1;
      //std::cout << "Matches size: " << (int)matches.size() << std::endl;
      for (int i = 0; (i < (int)matches.size()) && (classes_visited < LineMOD_Detector::num_classes); ++i)
      {
          //std::cout << "Matches size: " << (int)matches.size() << std::endl;
          //std::cout << i << std::endl;
          cv::linemod::Match m = matches[i];
          ROS_DEBUG("Matching count: %u", i);

          //std::cout << "I: " << i << "classes visited: " << classes_visited << std::endl;
          //std::cout << "matches.size: " << (int)matches.size() << "num classes: " <<
          //  LineMOD_Detector::num_classes << std::endl;
          if (m.similarity > best_match_similarity) {
              best_match_similarity = m.similarity;
              best_match_idx = i;
          }
          if (visited.insert(m.class_id).second)
          {
              ++classes_visited;
          }
      }
      bool sent_something=false;
      //std::cout << "Best match similarity: " << best_match_similarity << std::endl;
      //std::cout << "Best match idx: " << best_match_idx << std::endl;
      sensor_msgs::ImagePtr debugmsg;
      if (best_match_idx == -1) {
          if(_publish_debug_img)
          {
              debugmsg = cv_bridge::CvImage(color_ptr->header, color_ptr->encoding, display).toImageMsg();
          }
      }
      else
      {
          cv::linemod::Match m = matches[best_match_idx];

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

          if (m.class_id == "metal_tree" || m.class_id == "metal_star" ||
                  m.class_id == "metal_lines" || m.class_id == "metal_pi" ||
                  m.class_id == "metal_square") {
              hull = floodFillHull(hull, display);
          }

          if (dominant_color != "green") {
              drawResponse(templates, LineMOD_Detector::num_modalities, LineMOD_Detector::display, cv::Point(m.x, m.y), LineMOD_Detector::detector->getT(0));
          }

          if (m.similarity > LineMOD_Detector::pub_threshold && dominant_color!="green")
          {
              float angle;
              angle = computeGripAngle(hull);
              if (m.class_id == "red_puck" &&
                      (dominant_color=="red" || dominant_color=="pink" || dominant_color=="purple" || dominant_color=="orange"))
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::RED_PUCK);
              }
              if (m.class_id == "orange_pipe" &&
                      (dominant_color=="orange" || dominant_color=="white" || dominant_color=="yellow"))
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::ORANGE_PIPE);
              }
              if (m.class_id == "pre_cached" &&
                      (dominant_color=="white"))
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::PRE_CACHED);
              }
              if (m.class_id == "pre_cached_side" &&
                      (dominant_color=="white"))
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::PRE_CACHED);
              }
              if (m.class_id == "wood_cube" &&
                      (dominant_color=="yellow" || dominant_color=="brown" || dominant_color=="white"))
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::WOODEN_CUBE);
              }
              if (m.class_id == "pink_tennis_ball" &&
                      (dominant_color=="pink" || dominant_color=="white" || dominant_color=="red" ||
                       dominant_color=="orange" || dominant_color=="purple"))
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::PINK_TENNIS_BALL);
              }
              if (m.class_id == "colored_ball" &&
                      dominant_color!="brown" && dominant_color!="white" && dominant_color!="gray")
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::COLORED_BALL);
              }
              if (m.class_id == "metal_star")
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::METAL_1);
              }
              if (m.class_id == "metal_pi" )
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::METAL_1);
              }
              if (m.class_id == "metal_tree" )
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::METAL_1);
              }
              if (m.class_id == "metal_lines" )
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::METAL_1);
              }
              if (m.class_id == "metal_square" )
              {
                  sent_something = true;
                  LineMOD_Detector::publishPoint(templates, m, color_ptr->header,
                          angle, samplereturn_msgs::NamedPoint::METAL_1);
              }
          }
          debugmsg = cv_bridge::CvImage(color_ptr->header, color_ptr->encoding, display).toImageMsg();
      }

      // If no template, try BMS
      if(!sent_something && !hard_samples)
      {
          cv::Point bms_centriod;
          std::string bms_color;
          std::vector<cv::Point> hull;
          if(performBMSCheck(color_ptr->image, &bms_centriod, &bms_color, &hull))
          {
              float angle;
              angle = computeGripAngle(hull);
              ROS_DEBUG_STREAM("BMS success centroid: " << bms_centriod << " color: " << bms_color << " angle: " << angle);
          }
          else
          {
              ROS_DEBUG_STREAM("BMS fail");
          }
      }

      if(_publish_debug_img)
      {
          debug_img_pub.publish(debugmsg);
      }

      LineMOD_Detector::sources.clear();
  }

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
      cv::Mat trimmed = trimDisparity(flat, min_disp);
      ROS_DEBUG("Number of disparities in trimmed: %i",trimmed.cols);
      ROS_DEBUG("Number of disparities in flat: %i",flat.cols);
      if( trimmed.cols > 1 )
      {
          median_disp = trimmed.at<float>(0,(trimmed.cols/2));
      }
      else
      {
          ROS_ERROR("No disparities present, cannot emit point.");
          return;
      }
      ROS_DEBUG("Median Disp: %f", median_disp);

      cam_model_.projectDisparityTo3d(cv::Point2d(m.x+templates[1].width/2,m.y+templates[1].height/2),
          median_disp, xyz);
      temp_point.point.x = xyz.x;
      temp_point.point.y = xyz.y;
      ROS_DEBUG("Projected z: %f", xyz.z);
      if (xyz.z > max_depth) {
        temp_point.point.z = max_depth;
        temp_point.point.x /= (xyz.z/max_depth);
        temp_point.point.y /= (xyz.z/max_depth);
      }
      else if (xyz.z < min_depth) {
        temp_point.point.z = min_depth;
        temp_point.point.x /= (xyz.z/min_depth);
        temp_point.point.y /= (xyz.z/min_depth);
      }
      else {
        temp_point.point.z = xyz.z;
      }
      bool wait =
        listener_.waitForTransform(_detection_frame_id, header.frame_id, header.stamp, ros::Duration(0.03));
      if (!wait) {
        return;
      }
      listener_.transformPoint(_detection_frame_id, temp_point, odom_point);

      //std::cout << "Camera 3D point: " << temp_point << std::endl;
      point_msg.name = m.class_id;
      point_msg.sample_id = sample_id;
      point_msg.header = header;
      point_msg.header.frame_id = _detection_frame_id;
      point_msg.grip_angle = grip_angle;
      point_msg.point = odom_point.point;
      LineMOD_Detector::point_pub.publish(point_msg);
    }

    LineMOD_Detector::img_point_pub.publish(img_point_msg);
  }

  std::vector<cv::Point> floodFillHull(const std::vector<cv::Point>& hull,
      const cv::Mat color_image)
  {
    cv::Moments M = cv::moments(hull);
    float cx = M.m10/M.m00;
    float cy = M.m01/M.m00;
    std::vector<cv::Point> grip_hull;
    // Take the convex hull of a metal sample symbol, floodFill from some
    // points on it to get a mask for grip computation.
    for (int i = 0; i < 1; i++)
    {
      float offset_x = hull[i].x-cx;
      float offset_y = hull[i].y-cy;
      float length = std::sqrt(pow(offset_x,2)+pow(offset_y,2));
      cv::Point offset_pt((hull[i].x+10*(offset_x/length)),
                          (hull[i].y+10*(offset_y/length)));
      cv::Point trunc_offset_pt((offset_pt.x>=color_image.cols)?(color_image.cols-1):offset_pt.x,
                                (offset_pt.y>=color_image.rows)?(color_image.rows-1):offset_pt.y);
      cv::Mat mask;
      mask = cv::Mat::zeros(color_image.rows+2, color_image.cols+2, CV_8UC1);
      cv::floodFill(color_image, mask, trunc_offset_pt, cv::Scalar(255),
          0, cv::Scalar(8,8,8), cv::Scalar(8,8,8),
          (4|(255<<8)|CV_FLOODFILL_MASK_ONLY));
      //cv::imshow("mask",mask);
      //cv::waitKey(10);
      // Do some area bounds check, between 5x5cm and max gripper size (11x11cm)
      if(!maskToHull(mask, &grip_hull))
      {
          ROS_ERROR("Failed to find convex hull, no contours exist");
      }
    }
    return grip_hull;
  }

  bool maskToHull(cv::Mat mask, std::vector<cv::Point> *hull)
  {
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
      double maxArea = 0;
      int max_idx = 0;
      for (size_t i=0; i<contours.size(); i++) {
          double area = cv::contourArea(cv::Mat(contours[i]));
          if (area > maxArea) {
              maxArea = area;
              max_idx = i;
          }
      }
      if(contours.size()>0)
      {
          cv::convexHull(contours[max_idx], *hull);
          return true;
      }
      else
      {
          return false;
      }
  }

  cv::Mat trimDisparity(cv::Mat flat_disparity, float min_disparity) {
    int i = 0;
    while (i < flat_disparity.cols && flat_disparity.at<float>(0,i) < min_disparity) {
      i++;
    }
    return flat_disparity.colRange(i,flat_disparity.cols);
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

  bool performBMSCheck(const cv::Mat& image, cv::Point *bms_centroid, std::string *dominant_color,
          std::vector<cv::Point> *hull)
  {
      BMS bms(_config.bms_dilation_width_1, _config.bms_opening_width,
              _config.bms_normalize, _config.bms_handle_border);
      cv::Mat small;
      cv::resize(image,
              small,cv::Size(_config.bms_img_width,image.rows*(_config.bms_img_width/image.cols)),
              0.0,0.0,cv::INTER_AREA);

      bms.computeSaliency(small, _config.bms_sample_step);
      cv::Mat debug_bms_img = bms.getSaliencyMap().clone();

      if (_config.bms_thresh_on) {
          ROS_DEBUG("Thresholding");
          cv::threshold(debug_bms_img, debug_bms_img, _config.bms_thresh, 255, cv::THRESH_BINARY);
      }

      cv::SimpleBlobDetector::Params blob_params;
      blob_params.blobColor = _config.blobColor;
      blob_params.filterByColor = _config.filterByColor;
      blob_params.filterByArea = _config.filterByArea;
      blob_params.filterByConvexity = _config.filterByConvexity;
      blob_params.minDistBetweenBlobs = _config.minDistBetweenBlobs;
      blob_params.minArea = _config.minArea;
      blob_params.maxArea = _config.maxArea;
      blob_params.minConvexity = _config.minConvexity;
      blob_params.maxConvexity = _config.maxConvexity;
      blob_params.minThreshold = _config.minThreshold;
      blob_params.maxThreshold = _config.maxThreshold;
      blob_params.thresholdStep = _config.thresholdStep;
      blob_params.minRepeatability = _config.minRepeatability;

      cv::SimpleBlobDetector blob(blob_params);

      vector<cv::KeyPoint> kp;

      cv::Mat debug_bms_img_color;

      cv::Mat blob_copy = debug_bms_img.clone();
      blob.detect(blob_copy, kp);
      ROS_DEBUG("Keypoints Detected: %lu", kp.size());
      cv::cvtColor(debug_bms_img, debug_bms_img_color, CV_GRAY2RGB);
      cv::Mat sub_img, sub_mask;
      bool found = false;
      for (size_t i=0; i < kp.size(); i++)
      {
          cv::circle(debug_bms_img_color, kp[i].pt, 3*kp[i].size, CV_RGB(255,0,0), 1, 4);
          int x = kp[i].pt.x;
          int y = kp[i].pt.y;
          int size = 2*kp[i].size;
          int minrow, maxrow, mincol, maxcol;
          minrow = max(y-size,0);
          maxrow = min(y+size,small.rows);
          mincol = max(x-size,0);
          maxcol = min(x+size,small.cols);
          sub_img = small(Range(minrow, maxrow), Range(mincol, maxcol));
          sub_mask = debug_bms_img(Range(max(y-size,0), min(y+size,small.rows)), Range(max(x-size,0), min(x+size,small.cols)));
          if (cv::countNonZero(sub_mask) == 0) {
              ROS_DEBUG_STREAM("Skipping all zero blob at (" << x << ", " << y << ").");
              continue;
          }

          Eigen::Matrix<float,11,1> interiorColor(Eigen::Matrix<float,11,1>::Zero());
          Eigen::Matrix<float,11,1> exteriorColor(Eigen::Matrix<float,11,1>::Zero());
          exteriorColor = cn.computeExteriorColor(sub_img,sub_mask);
          interiorColor = cn.computeInteriorColor(sub_img,sub_mask,exteriorColor);

          *dominant_color = cn.getDominantColor(interiorColor);
          ROS_DEBUG_STREAM("Dominant color " << *dominant_color);
          string dominant_exterior_color = cn.getDominantColor(exteriorColor);
          ROS_DEBUG_STREAM("Dominant exterior color " << dominant_exterior_color);

          cv::putText(debug_bms_img_color, *dominant_color, kp[i].pt, FONT_HERSHEY_SIMPLEX, 0.5,
                  CV_RGB(100,100,100));

          std::vector<std::string>::iterator in_it, ex_it;
          in_it = std::find(interior_colors_vec_.begin(),interior_colors_vec_.end(),*dominant_color);
          ex_it = std::find(exterior_colors_vec_.begin(),exterior_colors_vec_.end(),dominant_exterior_color);

          if (in_it != interior_colors_vec_.end() &&
                  ex_it != exterior_colors_vec_.end()){
              float scale = image.cols/_config.bms_img_width;
              bms_centroid->x = kp[i].pt.x*scale;
              bms_centroid->y = kp[i].pt.y*scale;
              cv::circle(debug_bms_img_color, kp[i].pt, 2*kp[i].size, CV_RGB(0,0,255), 2, CV_AA);
              cv::putText(debug_bms_img_color, *dominant_color, kp[i].pt, FONT_HERSHEY_SIMPLEX, 0.5,
                      CV_RGB(0,255,0));
              ROS_DEBUG_STREAM( "BMS Coords: x: "<<kp[i].pt.x<<" y: "<<kp[i].pt.y );
              ROS_DEBUG_STREAM( "Pixel coords: x:" << kp[i].pt.x*scale <<"y: " << kp[i].pt.y*scale);
              found = true;
              if(maskToHull(sub_mask, hull))
              {
                  for(auto &pt : *hull)
                  {
                      pt.x += mincol;
                      pt.y += minrow;
                  }
                  std::vector<std::vector<cv::Point> > contours;
                  contours.push_back(*hull);
                  cv::drawContours(debug_bms_img_color, contours, 0, cv::Scalar(255,0,255));
                  break;
              }
          }
      }

      std_msgs::Header header;
      sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(header,"rgb8",debug_bms_img_color).toImageMsg();
      pub_bms_img.publish(debug_img_msg);

      return found;
  }

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "Line2D_Detector");
  LineMOD_Detector ld;
  ros::spin();
}
