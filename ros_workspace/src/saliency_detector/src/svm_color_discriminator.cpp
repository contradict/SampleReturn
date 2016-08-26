#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <samplereturn_msgs/PatchArray.h>
#include <samplereturn_msgs/NamedPoint.h>
#include <samplereturn_msgs/NamedPointArray.h>

#include <dynamic_reconfigure/server.h>
#include <saliency_detector/SVMColorDiscriminatorConfig.h>
#include <samplereturn/mask_utils.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/serialization/shared_ptr.hpp>

namespace saliency_detector {
// Take in a PatchArray, compute a color model of the salient object in each patch,
// publish a NamedPoint. This is the end of the detection part of sample search,
// objects past here have matched saliency, shape, size, and color criteria.
class SVMColorDiscriminator
{
  ros::Subscriber sub_patch_array;
  ros::Publisher pub_named_points;
  ros::Publisher pub_debug_image;

  dynamic_reconfigure::Server<saliency_detector::SVMColorDiscriminatorConfig> dr_srv;

  // OpenCV HSV represents H between 0-180, remember that
  saliency_detector::SVMColorDiscriminatorConfig config_;

  cv::Ptr<cv::ml::SVM> svm;
  cv::Mat mean_;
  cv::Mat std_;

  public:
  SVMColorDiscriminator() {
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<saliency_detector::SVMColorDiscriminatorConfig>::CallbackType cb;
    cb = boost::bind(&SVMColorDiscriminator::configCallback, this,  _1, _2);
    dr_srv.setCallback(cb);

    ros::NodeHandle private_node_handle("~");

    const std::string MODELFILE="model_file";
    std::string model_file;
    if(!private_node_handle.getParam(MODELFILE, model_file))
    {
        ROS_ERROR("No model file specified, exiting.");
        ros::requestShutdown();
        return;
    }

    svm = cv::ml::StatModel::load<cv::ml::SVM>(model_file,
            "opencv_ml_svm");
    if(!svm->isTrained())
    {
        ROS_ERROR("Loading model file %s failed, exiting", model_file.c_str());
        ros::requestShutdown();
        return;
    }
    cv::FileStorage fs(model_file, cv::FileStorage::READ);
    fs["mean"] >> mean_;
    fs["std"] >> std_;

    sub_patch_array =
      nh.subscribe("projected_patch_array", 1, &SVMColorDiscriminator::patchArrayCallback, this);

    pub_named_points =
      nh.advertise<samplereturn_msgs::NamedPointArray>("named_point", 1);

    pub_debug_image =
      nh.advertise<sensor_msgs::Image>("color_debug_image", 1);

  }

  void patchArrayCallback(const samplereturn_msgs::PatchArrayConstPtr& msg)
  {
    samplereturn_msgs::NamedPointArray points_out;
    points_out.header = msg->header;
    bool enable_debug = (pub_debug_image.getNumSubscribers() != 0);
    cv::Mat debug_image;
    if (enable_debug) {
      debug_image = cv::Mat::zeros(msg->cam_info.height,
          msg->cam_info.width, CV_8UC3);
    }
    for (size_t i = 0; i < msg->patch_array.size(); i++) {
      cv_bridge::CvImageConstPtr cv_ptr_mask, cv_ptr_img;
      sensor_msgs::ImageConstPtr msg_mask(&(msg->patch_array[i].mask), boost::serialization::null_deleter());
      sensor_msgs::ImageConstPtr msg_img(&(msg->patch_array[i].image), boost::serialization::null_deleter());
      try {
        cv_ptr_mask = cv_bridge::toCvShare(msg_mask, "mono8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge mask exception: %s", e.what());
      }
      try {
        cv_ptr_img = cv_bridge::toCvShare(msg_img, "rgb8");
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge image exception: %s", e.what());
      }
      if (enable_debug) {
        cv_ptr_img->image.copyTo(debug_image(cv::Rect(msg->patch_array[i].image_roi.x_offset,
                msg->patch_array[i].image_roi.y_offset,
                msg->patch_array[i].image_roi.width,
                msg->patch_array[i].image_roi.height)));
      }

      cv::Mat Lab;
      cv::cvtColor(cv_ptr_img->image, Lab, cv::COLOR_RGB2Lab);
      cv::Mat point(1,1,CV_32FC3);
      point = cv::mean(Lab, cv_ptr_mask->image);
      cv::Mat scaled_point = (point.reshape(1) - mean_)/std_;
      cv::Mat prediction;
      svm->predict(scaled_point, prediction, cv::ml::StatModel::RAW_OUTPUT);
      float distance = prediction.at<float>(0);

      if (enable_debug) {
          int x,y,h;
          x = msg->patch_array[i].image_roi.x_offset;
          y = msg->patch_array[i].image_roi.y_offset;
          //w = msg->patch_array[i].image_roi.width;
          h = msg->patch_array[i].image_roi.height;
          char edist[100];
          snprintf(edist, 100, "d:%3.2f", distance);
          cv::putText(debug_image,edist,cv::Point2d(x+70, y + h + 50*config_.debug_font_scale),
                  cv::FONT_HERSHEY_SIMPLEX, config_.debug_font_scale, cv::Scalar(255,0,0),4,cv::LINE_8);
      }

      bool is_sample = distance<0;

      if(!is_sample)
      {
          if(enable_debug)
          {
              // Nix region
              int x,y,w,h;
              x = msg->patch_array[i].image_roi.x_offset;
              y = msg->patch_array[i].image_roi.y_offset;
              w = msg->patch_array[i].image_roi.width;
              h = msg->patch_array[i].image_roi.height;
              cv::line(debug_image,
                      cv::Point2f(x, y),
                      cv::Point2f(x + w, y + h), cv::Scalar(255,0,0), 20);
              cv::line(debug_image,
                      cv::Point2f(x + w, y),
                      cv::Point2f(x, y + h), cv::Scalar(255,0,0), 20);
          }
          continue;
      }

      samplereturn_msgs::NamedPoint np_msg;

      if(config_.compute_grip_angle)
      {
          cv::RotatedRect griprect;
          if(samplereturn::computeGripAngle(cv_ptr_mask->image, &griprect, &np_msg.grip_angle) &&
                  enable_debug)
          {
              griprect.center += cv::Point2f(
                      msg->patch_array[i].image_roi.x_offset,
                      msg->patch_array[i].image_roi.y_offset);
              samplereturn::drawGripRect(debug_image, griprect);
          }
      }

      np_msg.header.stamp = msg->header.stamp;
      np_msg.header.frame_id = msg->patch_array[i].world_point.header.frame_id;
      np_msg.point = msg->patch_array[i].world_point.point;
      np_msg.sensor_frame = msg->header.frame_id;
      np_msg.name = "SVM";
      points_out.points.push_back(np_msg);
    }
    pub_named_points.publish(points_out);
    if (enable_debug) {
      sensor_msgs::ImagePtr debug_image_msg =
        cv_bridge::CvImage(msg->header,"rgb8",debug_image).toImageMsg();
      pub_debug_image.publish(debug_image_msg);
    }
  }

  void configCallback(saliency_detector::SVMColorDiscriminatorConfig &config, uint32_t level)
  {
      (void)level;
      config_ = config;
  }

};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svm_color_discriminator");
  saliency_detector::SVMColorDiscriminator svm;
  ros::spin();
}
