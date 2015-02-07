#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "gpuimageproc/stereoproc.h"

namespace gpuimageproc
{

void Stereoproc::onInit()
{
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    it_.reset(new image_transport::ImageTransport(nh));

    // Synchronize inputs. Topic subscriptions happen on demand in the connection
    // callback. Optionally do approximate synchronization.
    int queue_size;
    private_nh.param("queue_size", queue_size, 5);
    bool approx;
    private_nh.param("approximate_sync", approx, false);
    if (approx)
    {
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                    sub_l_mono_image_, sub_l_color_image_, sub_l_info_,
                    sub_r_mono_image_, sub_r_color_image_, sub_r_info_) );
        approximate_sync_->registerCallback(boost::bind(&Stereoproc::imageCb,
                    this, _1, _2, _3, _4, _5, _6));
    }
    else
    {
        exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                    sub_l_mono_image_, sub_l_color_image_, sub_l_info_,
                    sub_r_mono_image_, sub_r_color_image_, sub_r_info_) );
        exact_sync_->registerCallback(boost::bind(&Stereoproc::imageCb,
                    this, _1, _2, _3, _4, _5, _6));
    }

    // Set up dynamic reconfiguration
    ReconfigureServer::CallbackType f = boost::bind(&Stereoproc::configCb,
            this, _1, _2);
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
    reconfigure_server_->setCallback(f);

    // Monitor whether anyone is subscribed to the output
    ros::SubscriberStatusCallback connect_cb = boost::bind(&Stereoproc::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_mono_rect_left_ = nh.advertise<sensor_msgs::Image>("left/rect_mono", 1, connect_cb, connect_cb);
    pub_color_rect_left_ = nh.advertise<sensor_msgs::Image>("left/rect_color", 1, connect_cb, connect_cb);
    pub_mono_rect_right_ = nh.advertise<sensor_msgs::Image>("right/rect_mono", 1, connect_cb, connect_cb);
    pub_color_rect_right_ = nh.advertise<sensor_msgs::Image>("right/rect_color", 1, connect_cb, connect_cb);
    pub_disparity_ = nh.advertise<stereo_msgs::DisparityImage>("disparity", 1, connect_cb, connect_cb);
    pub_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, connect_cb, connect_cb);

    cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());
}

// Handles (un)subscribing when clients (un)subscribe
void Stereoproc::connectCb()
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    connected_.RectifyMonoLeft = (pub_mono_rect_left_.getNumSubscribers() > 0)?1:0;
    connected_.RectifyMonoRight = (pub_mono_rect_right_.getNumSubscribers() > 0)?1:0;
    connected_.RectifyColorLeft = (pub_color_rect_left_.getNumSubscribers() > 0)?1:0;
    connected_.RectifyColorRight = (pub_color_rect_right_.getNumSubscribers() > 0)?1:0;
    connected_.Disparity = (pub_disparity_.getNumSubscribers() > 0)?1:0;
    connected_.Pointcloud = (pub_pointcloud_.getNumSubscribers() > 0)?1:0;
    int level = connected_.level();
    if (level == 0)
    {
        sub_l_mono_image_.unsubscribe();
        sub_l_color_image_.unsubscribe();
        sub_l_info_.unsubscribe();
        sub_r_mono_image_.unsubscribe();
        sub_r_color_image_.unsubscribe();
        sub_r_info_.unsubscribe();
    }
    else if (!sub_l_mono_image_.getSubscriber())
    {
        ros::NodeHandle &nh = getNodeHandle();
        // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
        /// @todo Allow remapping left, right?
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_l_mono_image_.subscribe(*it_, "left/image_mono", 1, hints);
        sub_l_color_image_.subscribe(*it_, "left/image_color", 1, hints);
        sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
        sub_r_mono_image_.subscribe(*it_, "right/image_mono", 1, hints);
        sub_r_color_image_.subscribe(*it_, "right/image_color", 1, hints);
        sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
    }
}

class GPUSender
{
    public:
        GPUSender(sensor_msgs::ImageConstPtr example, std::string encoding,
                ros::Publisher* pub) :
            image_msg_(new sensor_msgs::Image()),
            publisher_(pub)
        {
            int bitdepth, channels;
            bitdepth = sensor_msgs::image_encodings::bitDepth(encoding);
            channels = sensor_msgs::image_encodings::numChannels(encoding);
            image_msg_->header = example->header;
            image_msg_->height = example->height;
            image_msg_->width = example->width;
            image_msg_->encoding = encoding;
            image_msg_->step = example->width*bitdepth*channels/8;
            image_msg_->data.resize( image_msg_->height * image_msg_->step );
            image_data_ = cv::Mat(image_msg_->height, image_msg_->width,
                    CV_MAKETYPE(CV_8U, channels),
                    &image_msg_->data[0], image_msg_->step);
            cv::gpu::registerPageLocked(image_data_);
        }
        ~GPUSender()
        {
            cv::gpu::unregisterPageLocked(image_data_);
        }
        void send(void)
        {
            publisher_->publish( image_msg_ );
        }
        void enqueueSend(cv::gpu::GpuMat& m, cv::gpu::Stream& strm)
        {
            strm.enqueueDownload(m, image_data_);
            strm.enqueueHostCallback(
                [](cv::gpu::Stream &strm, int status, void *userData)
                {
                   static_cast<GPUSender *>(userData)->send();
                },
                (void *)this);
        }
        typedef std::shared_ptr<GPUSender> Ptr;
    private:
        sensor_msgs::ImagePtr image_msg_;
        cv::Mat image_data_;
        ros::Publisher* publisher_;
};

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void Stereoproc::imageCb(
        const sensor_msgs::ImageConstPtr& l_mono_msg,
        const sensor_msgs::ImageConstPtr& l_color_msg,
        const sensor_msgs::CameraInfoConstPtr& l_info_msg,
        const sensor_msgs::ImageConstPtr& r_mono_msg,
        const sensor_msgs::ImageConstPtr& r_color_msg,
        const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    int level = connected_.level();
    NODELET_INFO("got images, level %d", level);

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    cv::gpu::Stream l_strm, r_strm;
    cv::gpu::GpuMat l_mono, r_mono;
    cv::gpu::GpuMat l_color, r_color;
    std::vector<GPUSender::Ptr> senders;

    // Create cv::Mat views onto all buffers
    const cv::Mat l_cpu_mono = cv_bridge::toCvShare(
            l_mono_msg,
            sensor_msgs::image_encodings::MONO8)->image;
    cv::gpu::registerPageLocked(const_cast<cv::Mat&>(l_cpu_mono));
    const cv::Mat l_cpu_color = cv_bridge::toCvShare(
            l_color_msg,
            sensor_msgs::image_encodings::BGR8)->image;
    cv::gpu::registerPageLocked(const_cast<cv::Mat&>(l_cpu_color));
    const cv::Mat r_cpu_mono = cv_bridge::toCvShare(
            r_mono_msg,
            sensor_msgs::image_encodings::MONO8)->image;
    cv::gpu::registerPageLocked(const_cast<cv::Mat&>(r_cpu_mono));
    const cv::Mat r_cpu_color = cv_bridge::toCvShare(
            r_color_msg,
            sensor_msgs::image_encodings::BGR8)->image;
    cv::gpu::registerPageLocked(const_cast<cv::Mat&>(r_cpu_color));
    l_strm.enqueueUpload(l_cpu_mono, l_mono);
    l_strm.enqueueUpload(l_cpu_color, l_color);
    r_strm.enqueueUpload(r_cpu_mono, r_mono);
    if(connected_.RectifyColorRight)
        r_strm.enqueueUpload(r_cpu_color, r_color);

    cv::gpu::GpuMat l_rect_mono, r_rect_mono;
    if(connected_.RectifyMonoLeft || connected_.Disparity || connected_.Pointcloud)
    {
        model_.left().rectifyImageGPU(l_mono, l_rect_mono, l_strm);
    }
    if(connected_.RectifyMonoRight || connected_.Disparity || connected_.Pointcloud)
    {
        model_.right().rectifyImageGPU(r_mono, r_rect_mono, r_strm);
    }

    if(connected_.RectifyMonoLeft)
    {
       GPUSender::Ptr t(new GPUSender(l_mono_msg, sensor_msgs::image_encodings::MONO8, &pub_mono_rect_left_));
       senders.push_back(t);
       t->enqueueSend(l_rect_mono, l_strm);
    }

    if(connected_.RectifyMonoRight)
    {
       GPUSender::Ptr t(new GPUSender(r_mono_msg, sensor_msgs::image_encodings::MONO8, &pub_mono_rect_right_));
       senders.push_back(t);
       t->enqueueSend(r_rect_mono, r_strm);
    }

    cv::gpu::GpuMat l_rect_color, r_rect_color;
    if(connected_.RectifyColorLeft || connected_.Pointcloud)
    {
        model_.left().rectifyImageGPU(l_color, l_rect_color, l_strm);
    }
    if(connected_.RectifyColorRight)
    {
        model_.right().rectifyImageGPU(r_color, r_rect_color, r_strm);
    }

    if(connected_.RectifyColorLeft)
    {
       GPUSender::Ptr t(new GPUSender(l_color_msg, sensor_msgs::image_encodings::BGR8, &pub_color_rect_left_));
       senders.push_back(t);
       t->enqueueSend(l_rect_color, l_strm);
    }

    if(connected_.RectifyColorRight)
    {
       GPUSender::Ptr t(new GPUSender(r_color_msg, sensor_msgs::image_encodings::BGR8, &pub_color_rect_right_));
       senders.push_back(t);
       t->enqueueSend(r_rect_color, r_strm);
    }

    cv::gpu::GpuMat disparity;
    if(connected_.Disparity || connected_.Pointcloud)
    {
        // May need to wait for r_strm completion here
        block_matcher_(l_rect_mono, r_rect_mono, disparity, l_strm);
    }

    if(connected_.Disparity)
    {
        cv::gpu::GpuMat disparity_float;
        l_strm.enqueueConvert(disparity, disparity_float, CV_32FC1);

        // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
        double cx_l = model_.left().cx();
        double cx_r = model_.right().cx();
        if (cx_l != cx_r)
        {
            cv::gpu::subtract(disparity_float, cv::Scalar(cx_l - cx_r),
                    disparity_float, cv::gpu::GpuMat(), -1, l_strm);
        }

        // Allocate new disparity image message
        disp_msg_.reset(new stereo_msgs::DisparityImage());
        disp_msg_->header         = l_info_msg->header;
        disp_msg_->image.header   = l_info_msg->header;

        // Compute window of (potentially) valid disparities
        int border   = block_matcher_.winSize / 2;
        int left   = block_matcher_.ndisp + block_matcher_min_disparity_ + border - 1;
        int wtf = (block_matcher_min_disparity_ >= 0) ? border + block_matcher_min_disparity_ : std::max(border, -block_matcher_min_disparity_);
        int right  = disp_msg_->image.width - 1 - wtf;
        int top    = border;
        int bottom = disp_msg_->image.height - 1 - border;
        disp_msg_->valid_window.x_offset = left;
        disp_msg_->valid_window.y_offset = top;
        disp_msg_->valid_window.width    = right - left;
        disp_msg_->valid_window.height   = bottom - top;
        disp_msg_->min_disparity = block_matcher_min_disparity_;
        disp_msg_->max_disparity = block_matcher_min_disparity_ + block_matcher_.ndisp - 1;

        disp_msg_->image.height = l_rect_mono.rows;
        disp_msg_->image.width = l_rect_mono.cols;
        disp_msg_->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        disp_msg_->image.step = l_rect_mono.cols * sizeof(float);
        disp_msg_->image.data.resize(disp_msg_->image.step*disp_msg_->image.height);
        disp_msg_data_ = cv::Mat_<float> (disp_msg_->image.height,
                             disp_msg_->image.width,
                             (float *)&disp_msg_->image.data[0],
                             disp_msg_->image.step);
        cv::gpu::registerPageLocked(disp_msg_data_);

        l_strm.enqueueDownload(disparity_float, disp_msg_data_);

        l_strm.enqueueHostCallback(
                [](cv::gpu::Stream &strm, int status, void *userData)
                {
                    static_cast<Stereoproc*>(userData)->sendDisparity();
                },
                (void*)this);
    }

    cv::gpu::GpuMat xyzw;
    if(connected_.Pointcloud)
    {
        model_.projectDisparityImageTo3dGPU(disparity, xyzw, true, l_strm);
        cv::gpu::CudaMem cuda_xyzw;
        l_strm.enqueueDownload(xyzw, cuda_xyzw);
        sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
        points_msg->header = l_mono_msg->header;
        points_msg->height = xyzw.rows;
        points_msg->width  = xyzw.cols;
        points_msg->fields.resize (4);
        points_msg->fields[0].name = "x";
        points_msg->fields[0].offset = 0;
        points_msg->fields[0].count = 1;
        points_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        points_msg->fields[1].name = "y";
        points_msg->fields[1].offset = 4;
        points_msg->fields[1].count = 1;
        points_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        points_msg->fields[2].name = "z";
        points_msg->fields[2].offset = 8;
        points_msg->fields[2].count = 1;
        points_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        points_msg->fields[3].name = "rgb";
        points_msg->fields[3].offset = 12;
        points_msg->fields[3].count = 1;
        points_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        //points_msg->is_bigendian = false; ???
        static const int STEP = 16;
        points_msg->point_step = STEP;
        points_msg->row_step = points_msg->point_step * points_msg->width;
        points_msg->data.resize (points_msg->row_step * points_msg->height);
        points_msg->is_dense = false; // there may be invalid points

        l_strm.waitForCompletion();
        cv::Mat_<cv::Vec3f> cpu_xyzw = cuda_xyzw.createMatHeader();
        float bad_point = std::numeric_limits<float>::quiet_NaN ();
        int offset = 0;
        for (int v = 0; v < cpu_xyzw.rows; ++v)
        {
            for (int u = 0; u < cpu_xyzw.cols; ++u, offset += STEP)
            {
                if (isValidPoint(cpu_xyzw(v,u)))
                {
                    // x,y,z,rgba
                    memcpy (&points_msg->data[offset + 0], &cpu_xyzw(v,u)[0], sizeof (float));
                    memcpy (&points_msg->data[offset + 4], &cpu_xyzw(v,u)[1], sizeof (float));
                    memcpy (&points_msg->data[offset + 8], &cpu_xyzw(v,u)[2], sizeof (float));
                }
                else
                {
                    memcpy (&points_msg->data[offset + 0], &bad_point, sizeof (float));
                    memcpy (&points_msg->data[offset + 4], &bad_point, sizeof (float));
                    memcpy (&points_msg->data[offset + 8], &bad_point, sizeof (float));
                }
            }
        }

        // Fill in color
        namespace enc = sensor_msgs::image_encodings;
        const std::string& encoding = l_color_msg->encoding;
        offset = 0;
        if(encoding == enc::RGB8)
        {
            const cv::Mat_<cv::Vec3b> color(l_color_msg->height, l_color_msg->width,
                    (cv::Vec3b*)&l_color_msg->data[0],
                    l_color_msg->step);
            for (int v = 0; v < cpu_xyzw.rows; ++v)
            {
                for (int u = 0; u < cpu_xyzw.cols; ++u, offset += STEP)
                {
                    if (isValidPoint(cpu_xyzw(v,u)))
                    {
                        const cv::Vec3b& rgb = color(v,u);
                        int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
                        memcpy (&points_msg->data[offset + 12], &rgb_packed, sizeof (int32_t));
                    }
                    else
                    {
                        memcpy (&points_msg->data[offset + 12], &bad_point, sizeof (float));
                    }
                }
            }
        }
        else if (encoding == enc::BGR8)
        {
            const cv::Mat_<cv::Vec3b> color(l_color_msg->height, l_color_msg->width,
                    (cv::Vec3b*)&l_color_msg->data[0],
                    l_color_msg->step);
            for (int v = 0; v < cpu_xyzw.rows; ++v)
            {
                for (int u = 0; u < cpu_xyzw.cols; ++u, offset += STEP)
                {
                    if (isValidPoint(cpu_xyzw(v,u)))
                    {
                        const cv::Vec3b& bgr = color(v,u);
                        int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
                        memcpy (&points_msg->data[offset + 12], &rgb_packed, sizeof (int32_t));
                    }
                    else
                    {
                        memcpy (&points_msg->data[offset + 12], &bad_point, sizeof (float));
                    }
                }
            }
        }
        else
        {
            NODELET_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
                    "unsupported encoding '%s'", encoding.c_str());
        }

        pub_pointcloud_.publish(points_msg);

    }

    l_strm.waitForCompletion();
    r_strm.waitForCompletion();
    if(connected_.Disparity)
        cv::gpu::unregisterPageLocked(disp_msg_data_);
    cv::gpu::unregisterPageLocked( const_cast<cv::Mat&>(l_cpu_mono) );
    cv::gpu::unregisterPageLocked( const_cast<cv::Mat&>(l_cpu_color) );
    cv::gpu::unregisterPageLocked( const_cast<cv::Mat&>(r_cpu_mono) );
    cv::gpu::unregisterPageLocked( const_cast<cv::Mat&>(r_cpu_color) );
}

void Stereoproc::sendDisparity(void)
{
    pub_disparity_.publish(disp_msg_);
}

void Stereoproc::configCb(Config &config, uint32_t level)
{
    // Tweak all settings to be valid
    config.correlation_window_size |= 0x1; // must be odd
    config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

    // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
    // concurrently, so this is thread-safe.
    block_matcher_.preset = config.preset;
    block_matcher_.winSize = config.correlation_window_size;
    block_matcher_.ndisp = config.disparity_range;
    block_matcher_.avergeTexThreshold = config.texture_threshold;
}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( gpuimageproc::Stereoproc, nodelet::Nodelet)
