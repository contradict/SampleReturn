#include <ros/ros.h>
#include <ros/console.h>
#include <algorithm>

#include <samplereturn_msgs/NamedPoint.h>
#include <samplereturn_msgs/PatchArray.h>
#include <pcl_msgs/ModelCoefficients.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <tf/transform_listener.h>
#include <linemod_detector/LinemodConfig.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include "new_modalities.hpp"
#include <saliency_detector/colormodel.h>
#include <linemod_detector/mask_utils.h>

namespace linemod_detector {
// Function prototypes
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

class LineMOD_Detector
{
  ros::Subscriber patch_array_sub;
  ros::Subscriber plane_model_sub;
  ros::Publisher point_pub;
  ros::Publisher debug_img_pub;
  ros::Publisher debug_mask_pub;
  dynamic_reconfigure::Server<linemod_detector::LinemodConfig> reconfigure;
  cv::Ptr<cv::linemod::Detector> detector;
  int num_modalities;
  int flood_fill_offset_;
  std::string template_filename;

  tf::Stamped<tf::Vector3> ground_normal_;

  std::vector<std::string> interior_colors_vec_, exterior_colors_vec_;

  linemod_detector::LinemodConfig _config;

  tf::TransformListener listener_;
  ros::ServiceServer service_;

  public:
  LineMOD_Detector()
  {
    ros::NodeHandle nh;
    point_pub = nh.advertise<samplereturn_msgs::NamedPoint>("point", 1);

    ros::NodeHandle pnh("~");
    debug_img_pub = pnh.advertise<sensor_msgs::Image>("linemod_2d_debug_img", 1);
    debug_mask_pub = pnh.advertise<sensor_msgs::Image>("linemod_2d_debug_mask", 1);

    bool load_inner_linemod;
    ros::param::param<std::string>("~template_file", template_filename,
            "/home/zlizer/src/SampleReturn/ros_workspace/src/linemod_detector/config/metal_samples.yaml");
    ros::param::param<bool>("~load_inner_linemod", load_inner_linemod, false);

    reconfigure.setCallback(boost::bind(&LineMOD_Detector::configCallback, this,  _1, _2));

    // Initialize LINEMOD data structures
    if (load_inner_linemod) {
      detector = readInnerLinemod(template_filename);
      num_modalities = (int)detector->getModalities().size();
    }
    else {
      detector = readLinemod(template_filename);
      num_modalities = (int)detector->getModalities().size();
    }
    ROS_DEBUG("Number of modalities loaded: %i", num_modalities);

    ground_normal_ = tf::Stamped<tf::Vector3>(tf::Vector3(0,0,1.),ros::Time(0),"base_link");
    patch_array_sub = nh.subscribe("projected_patch_array", 1, &LineMOD_Detector::patchArrayCallback, this);
    plane_model_sub = nh.subscribe("plane_model", 1, &LineMOD_Detector::planeModelCallback, this);

  }

  void planeModelCallback(const pcl_msgs::ModelCoefficientsPtr& msg)
  {
    if (msg->values.size() != 4) {
      ROS_DEBUG("Invalid Plane Fit");
    }
    else {
      ground_normal_.setData(tf::Vector3(msg->values[0], msg->values[1], msg->values[2]));
      ground_normal_.stamp_ = msg->header.stamp;
      ground_normal_.frame_id_ = msg->header.frame_id;
    }
  }

  void configCallback(linemod_detector::LinemodConfig &config, uint32_t level)
  {
      (void)level;
      _config = config;
  }

  cv::Mat rectifyPatch(std::string frame_id, const Eigen::Matrix3d& K, cv::Rect roi, const cv::Mat& img, int tw, int th)
  {
    // Take ground plane and patch camera info, compute rotation matrix
    // to ground, get square ground patch

    // Get ground plane homography
    tf::Stamped<tf::Vector3> ground_normal_cam;
    try {
      listener_.transformVector(frame_id, ground_normal_, ground_normal_cam);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ground_normal_.setData(tf::Vector3(0,0,1.));
      ground_normal_.stamp_ = ros::Time(0);
      ground_normal_.frame_id_ = "base_link";
    }
    Eigen::Vector3d eigen_cam_axis, eigen_ground_normal_cam;
    eigen_cam_axis << 0,0,-1;
    tf::vectorTFToEigen(ground_normal_cam, eigen_ground_normal_cam);
    Eigen::Matrix3d R = Eigen::Quaterniond().FromTwoVectors(eigen_cam_axis,
        eigen_ground_normal_cam).toRotationMatrix();
    Eigen::Matrix3d H = K*R*K.inverse();
    cv::Mat H_cv(3,3,CV_64F);
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
        H_cv.at<double>(i,j) = H(i,j);
      }
    }

    // Transform image points to ground
    int x,y,w,h;
    x = roi.x;
    y = roi.y;
    w = roi.width;
    h = roi.height;
    std::vector<cv::Point2f> img_points(4);
    img_points[0] = cv::Point(x, y);
    img_points[1] = cv::Point(x, y+h);
    img_points[2] = cv::Point(x+w, y+h);
    img_points[3] = cv::Point(x+w, y);
    std::vector<cv::Point2f> ground_points(4);
    cv::perspectiveTransform(img_points, ground_points, H_cv.inv());

    // Square up the ground points
    double sq_x = ground_points[0].x;
    double width = fabs(ground_points[1].x - ground_points[2].x);
    double mid_y = (ground_points[0].y + ground_points[1].y)/2.;
    std::vector<cv::Point2f> sq_ground_points(4);
    sq_ground_points[0] = cv::Point(sq_x, mid_y-width/2.);
    sq_ground_points[1] = cv::Point(sq_x, mid_y+width/2.);
    sq_ground_points[2] = cv::Point(sq_x + width, mid_y+width/2.);
    sq_ground_points[3] = cv::Point(sq_x + width, mid_y-width/2.);

    // Project them back into the image
    std::vector<cv::Point2f> sq_img_points(4);
    cv::perspectiveTransform(sq_ground_points, sq_img_points, H_cv);

    // Remove RoI offset
    for (size_t i=0; i<sq_img_points.size(); i++) {
      sq_img_points[i].x -= roi.x;
      sq_img_points[i].y -= roi.y;
    }

    // Reproject the region into the target square
    std::vector<cv::Point2f> out_win_points(4);
    out_win_points[0] = cv::Point(0, 0);
    out_win_points[1] = cv::Point(0, th);
    out_win_points[2] = cv::Point(tw, th);
    out_win_points[3] = cv::Point(tw, 0);
    cv::Mat win_transform = cv::getPerspectiveTransform(sq_img_points, out_win_points);
    cv::Mat out_win;
    cv::warpPerspective(img, out_win, win_transform, cv::Size(tw, th));
    return out_win;
  }

  void patchArrayCallback(const samplereturn_msgs::PatchArrayConstPtr& msg)
  {
    // The patch array will have some images and saliency masks, and will
    // already have the world point projected. Resize image to fixed size,
    // check for a LineMOD match, background color difference, and publish a
    // NamedPoint if it's good.
    if (msg->patch_array.empty()) {
      return;
    }

    cv::Mat debug_image = cv::Mat::zeros(msg->cam_info.height,
        msg->cam_info.width, CV_8UC3);

    cv_bridge::CvImagePtr cv_ptr_mask, cv_ptr_img;
    for (size_t i = 0; i < msg->patch_array.size(); i++) {
      try {
        cv_ptr_mask = cv_bridge::toCvCopy(msg->patch_array[i].mask, "mono8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge mask exception: %s", e.what());
      }
      try {
        cv_ptr_img = cv_bridge::toCvCopy(msg->patch_array[i].image, "rgb8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge image exception: %s", e.what());
      }

      // Get original patch size and location
      int orig_height, orig_width, orig_x, orig_y;
      orig_height = msg->patch_array[i].image_roi.height;
      orig_width = msg->patch_array[i].image_roi.width;
      orig_x = msg->patch_array[i].image_roi.x_offset;
      orig_y = msg->patch_array[i].image_roi.y_offset;

      // Add a mask to this
      cv::Mat det_img = cv::Mat::zeros(480, 640, CV_8UC3);
      cv::Mat det_mask = cv::Mat::zeros(480, 640, CV_8UC1);

      int w = _config.target_width;
      int h = _config.target_width * msg->patch_array[i].image_roi.height/
          msg->patch_array[i].image_roi.width;
      cv::Rect boundingbox;
      if(computeBoundingBox(cv_ptr_mask->image, &boundingbox))
      {
          double s=msg->patch_array[i].image_roi.width/double(boundingbox.size().width);
          w *= s;
          h *= s;
      }
      if(w>640)
      {
          h = h*(640.0/w);
          w = 640;
      }
      if(h>480)
      {
          w = w*(480.0/h);
          h = 480;
      }

      if(_config.rectify_patch)
      {
          Eigen::Matrix3d K = Eigen::Matrix3d::Map(msg->cam_info.K.data());
          K(0,2) = K(2,0);
          K(1,2) = K(2,1);
          K(2,0) = 0;
          K(2,1) = 0;

          cv::Rect roi(msg->patch_array[i].image_roi.x_offset,
                       msg->patch_array[i].image_roi.y_offset,
                       msg->patch_array[i].image_roi.width,
                       msg->patch_array[i].image_roi.height);

          // Rectify to get "head-on" view, constant size
          cv::Mat det_patch_img = rectifyPatch(msg->header.frame_id, K, roi,
                  cv_ptr_img->image,
                  w, h);
          cv::Mat det_patch_mask = rectifyPatch(msg->header.frame_id, K, roi,
                  cv_ptr_mask->image,
                  w, h);
          // Place in 640x480 image to match templates, otherwise templates can
          // run off edge.
          det_patch_img.copyTo(det_img(cv::Rect(0, 0, w, h)));
          det_patch_mask.copyTo(det_mask(cv::Rect(0, 0, w, h)));

      }
      else
      {
          cv::resize(cv_ptr_img->image,
                  det_img( cv::Rect( 0, 0, w, h) ),
                  cv::Size(w, h),
                  0, 0, cv::INTER_AREA);

          cv::resize(cv_ptr_mask->image,
                  det_mask( cv::Rect( 0, 0, w, h) ),
                  cv::Size(w, h),
                  0, 0, cv::INTER_AREA);
      }

      cv::dilate(det_mask, det_mask, cv::Mat(), cv::Point(-1,-1), _config.mask_dilation_iterations);

      // Detect
      std::vector<cv::Mat> sources;
      std::vector<cv::Mat> masks;
      if (_config.do_median_blur) {
          cv::Mat blur;
          cv::medianBlur(det_img, blur, _config.median_blur_size);
          blur.copyTo(det_img);
      }
      sources.push_back(det_img);
      masks.push_back(det_mask);
      cv::linemod::Match m;
      bool ret = matchLineMOD(sources, masks, m);

      if (!ret) {
        continue;
      }

      const std::vector<cv::linemod::Template>& templates = LineMOD_Detector::detector->getTemplates(m.class_id, m.template_id);

      // Draw Response on output image
      cv::Rect draw_rect; 
      if (debug_img_pub.getNumSubscribers()>0) {
        int draw_x_off = 0; int draw_y_off = 0;
        int draw_w_off = 0; int draw_h_off = 0;
        draw_rect = cv::Rect(orig_x + orig_width/2. - w/2.,
            orig_y + orig_height/2. - h/2.,
            w,
            h);
        if (draw_rect.x < 0) {
          draw_x_off = -draw_rect.x;
          draw_rect.x = 0;
        }
        if (draw_rect.y < 0) {
          draw_y_off = -draw_rect.y;
          draw_rect.y = 0;
        }
        if (draw_rect.x + draw_rect.width > debug_image.cols) {
          draw_w_off = draw_rect.x + draw_rect.width - debug_image.cols;
          draw_rect.width -= draw_w_off;
        }
        if (draw_rect.y + draw_rect.height > debug_image.rows) {
          draw_h_off = draw_rect.y + draw_rect.height - debug_image.rows;
          draw_rect.height -= draw_h_off;
        }

        drawResponse(templates, LineMOD_Detector::num_modalities, det_img,
            cv::Point(m.x, m.y), LineMOD_Detector::detector->getT(0), m.similarity);
        // Place in output image
        det_img(cv::Rect(draw_x_off,
                         draw_y_off,
                         w - draw_w_off,
                         h - draw_h_off)).copyTo(
                            debug_image(draw_rect),
                            det_mask(cv::Rect(draw_x_off,
                                              draw_y_off,
                                              w - draw_w_off,
                                              h - draw_h_off)));

      }

      // If a positive match, publish NamedPoint
      if (m.similarity < _config.pub_threshold)
      {
          continue;
      }

      // Do a background/foreground color histogram comparison, drop if
      // too similar
      if(_config.check_color_model)
      {
          saliency_detector::ColorModel cm(det_img, det_mask);
          saliency_detector::HueHistogram hh_inner = cm.getInnerHueHistogram(_config.min_color_saturation, _config.low_saturation_limit, _config.high_saturation_limit);
          saliency_detector::HueHistogram hh_outer = cm.getOuterHueHistogram(_config.min_color_saturation, _config.low_saturation_limit, _config.high_saturation_limit);
          double d_background = hh_inner.distance(hh_outer);
          double d_value = cm.getValuedSampleModel(_config.min_color_saturation, _config.low_saturation_limit, _config.high_saturation_limit).distance(hh_inner);
          if(debug_img_pub.getNumSubscribers()>0)
          {
              hh_inner.draw_histogram(debug_image, draw_rect.x, draw_rect.y+draw_rect.height);
          }
          if( (d_background<_config.min_inner_outer_distance) ||
              (d_value>_config.max_exemplar_distance))
          {
              continue;
          }
      }
 
      samplereturn_msgs::NamedPoint np;
      np.header = msg->header;
      np.point = msg->patch_array[i].world_point.point;
      if(_config.compute_grip_angle)
      {
          cv::RotatedRect griprect;
          if(computeGripAngle(det_mask, &griprect))
          {
              if(griprect.size.width>griprect.size.height)
              {
                  np.grip_angle = -griprect.angle*M_PI/180+M_PI_2;
              }
              else
              {
                  np.grip_angle = -griprect.angle*M_PI/180;
              }
              np.grip_angle = unwrap90(np.grip_angle);
              ROS_DEBUG("reported angle: %f", np.grip_angle);

              np.grip_angle = griprect.angle;
              if(debug_img_pub.getNumSubscribers()>0)
              {
                  griprect.center += cv::Point2f(orig_x + orig_width/2 - w/2,
                          orig_y + orig_height/2 - h/2);
                  cv::Mat pts(4,1,CV_32FC2);
                  griprect.points((cv::Point2f*)pts.data);
                  pts.convertTo( pts, CV_32S);
                  cv::polylines(debug_image, (cv::Point2i**)&pts.data, &pts.rows, 1, true, cv::Scalar(255,0,0), 3, CV_AA, 0);
              }
          }
      }
      point_pub.publish(np);
    }

    if(debug_img_pub.getNumSubscribers() > 0)
    {
        sensor_msgs::ImagePtr debug_image_msg =
            cv_bridge::CvImage(msg->header,"rgb8",debug_image).toImageMsg();
        debug_img_pub.publish(debug_image_msg);
    }
  }

  bool matchLineMOD(std::vector<cv::Mat> sources, std::vector<cv::Mat> masks, cv::linemod::Match &best_match)
  {
      // Perform matching
      std::vector<cv::linemod::Match> matches;
      std::vector<cv::String> class_ids;
      std::vector<cv::Mat> quantized_images;

      detector->match(sources, _config.matching_threshold, matches, class_ids, quantized_images, masks);

      int num_classes = detector->numClasses();
      ROS_DEBUG("Num Classes: %u", num_classes);
      int classes_visited = 0;
      std::set<std::string> visited;

      ROS_DEBUG("Matches size: %u", (int)matches.size());
      float best_match_similarity = 0;
      int best_match_idx = -1;
      for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
      {
          cv::linemod::Match m = matches[i];
          if (m.similarity > best_match_similarity) {
              best_match_similarity = m.similarity;
              best_match_idx = i;
          }
          if (visited.insert(m.class_id).second)
          {
              ++classes_visited;
          }
      }

      if (best_match_idx == -1) {
          ROS_DEBUG("No linemode matches found");
        return false;
      }
      else {
        best_match = matches[best_match_idx];
        ROS_DEBUG("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
                best_match.similarity, best_match.x, best_match.y,
                best_match.class_id.c_str(), best_match.template_id);
        return true;
      }
  }


  double unwrap90(double angle)
  {
      while(angle>M_PI/2)
          angle -= M_PI;
      while(angle<-M_PI/2)
          angle += M_PI;
      return angle;
  }

  void drawResponse(const std::vector<cv::linemod::Template>& templates,
                    int num_modalities, cv::Mat& dst, cv::Point offset, int T,
                    double similarity)
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
      cv::Scalar color = COLORS[2];
      if (similarity > _config.pub_threshold) {
        color = COLORS[0];
      }
      else if (similarity > _config.matching_threshold) {
        color = COLORS[5];
      }
      else {
        ROS_ERROR("Call to drawResponse with below matching response");
      }

      for (int i = 0; i < (int)templates[m].features.size(); ++i)
      {
        cv::linemod::Feature f = templates[m].features[i];
        cv::Point pt(f.x + offset.x, f.y + offset.y);
        cv::circle(dst, pt, T / 2, color);
      }
    }
  }

};
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "Line2D_Detector");
  linemod_detector::LineMOD_Detector ld;
  ros::spin();
}
