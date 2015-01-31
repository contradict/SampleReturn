#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <gpuimageproc/stereoproc.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>


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
                    sub_l_image_, sub_l_info_,
                    sub_r_image_, sub_r_info_) );
        approximate_sync_->registerCallback(boost::bind(&Stereoproc::imageCb,
                    this, _1, _2, _3, _4));
    }
    else
    {
        exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                    sub_l_image_, sub_l_info_,
                    sub_r_image_, sub_r_info_) );
        exact_sync_->registerCallback(boost::bind(&Stereoproc::imageCb,
                    this, _1, _2, _3, _4));
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
    pub_mono_ = nh.advertise<sensor_msgs::Image>("image_mono", 1, connect_cb, connect_cb);
    pub_color_ = nh.advertise<sensor_msgs::Image>("image_color", 1, connect_cb, connect_cb);
    pub_mono_rect_ = nh.advertise<sensor_msgs::Image>("image_mono_rect", 1, connect_cb, connect_cb);
    pub_color_rect_ = nh.advertise<sensor_msgs::Image>("image_color_rect", 1, connect_cb, connect_cb);
    pub_disparity_ = nh.advertise<stereo_msgs::DisparityImage>("disparity", 1, connect_cb, connect_cb);
    pub_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void Stereoproc::connectCb()
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    connected.DebayerMono = (pub_mono_.getNumSubscribers() > 0)?1:0;
    connected.DebayerColor = (pub_color_.getNumSubscribers() > 0)?1:0;
    connected.RectifyMono = (pub_mono_rect_.getNumSubscribers() > 0)?1:0;
    connected.RectifyColor = (pub_color_rect_.getNumSubscribers() > 0)?1:0;
    connected.Disparity = (pub_disparity_.getNumSubscribers() > 0)?1:0;
    connected.Pointcloud = (pub_pointcloud_.getNumSubscribers() > 0)?1:0;
    int level = connected.level();
    if (level == 0)
    {
        sub_l_image_.unsubscribe();
        sub_l_info_ .unsubscribe();
        sub_r_image_.unsubscribe();
        sub_r_info_ .unsubscribe();
    }
    else if (!sub_l_image_.getSubscriber())
    {
        ros::NodeHandle &nh = getNodeHandle();
        // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
        /// @todo Allow remapping left, right?
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
        sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
        sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
        sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
    }
}

void Stereoproc::imageCb(const sensor_msgs::ImageConstPtr& l_image_msg,
        const sensor_msgs::CameraInfoConstPtr& l_info_msg,
        const sensor_msgs::ImageConstPtr& r_image_msg,
        const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    int level = connected.level();

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    cv::gpu::Stream l_strm, r_strm;
    cv::gpu::GpuMat l_raw, r_raw;
    if(level>0) {
        // Create cv::Mat views onto all buffers
        const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
        const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;
        l_strm.enqueueUpload(l_image, l_raw);
        r_strm.enqueueUpload(r_image, r_raw);
    }

    cv::gpu::GpuMat l_mono, r_mono;
    if(connected.DebayerMono || connected.RectifyMono  || connected.Disparity || connected.Pointcloud)
    {
        // need mono rectified image
        cv::gpu::cvtColor(l_raw, l_mono, CV_BayerRG2GRAY, 0, l_strm);
        cv::gpu::cvtColor(r_raw, r_mono, CV_BayerRG2GRAY, 0, r_strm);
    }

    cv::gpu::GpuMat l_color, r_color;
    if(connected.DebayerColor || connected.Pointcloud)
    {
        // need color rectified image
        cv::gpu::cvtColor(l_raw, l_mono, CV_BayerRG2BGR, 0, l_strm);
        cv::gpu::cvtColor(r_raw, r_mono, CV_BayerRG2BGR, 0, r_strm);
    }

    cv::gpu::GpuMat l_rect_mono, r_rect_mono;
    if(connected.RectifyMono || connected.Disparity || connected.Pointcloud)
    {
        model_.left().rectifyImageGPU(l_mono, l_rect_mono, l_strm);
        model_.right().rectifyImageGPU(r_mono, r_rect_mono, r_strm);
    }
    cv::gpu::GpuMat l_rect_color, r_rect_color;
    if(connected.RectifyColor || connected.Pointcloud)
    {
        model_.left().rectifyImageGPU(l_color, l_rect_color, l_strm);
        model_.right().rectifyImageGPU(r_color, r_rect_color, r_strm);
    }

    cv::gpu::GpuMat disparity;
    if(connected.Disparity || connected.Pointcloud)
    {
        cv::gpu::StereoBM_GPU bm;
        r_strm.waitForCompletion();
        bm(l_rect_mono, r_rect_mono, disparity, l_strm);
    }

    cv::gpu::GpuMat xyzw;
    if(connected.Pointcloud)
    {
        model_.projectDisparityImageTo3dGPU(disparity, xyzw, true, l_strm);
    }

    // Allocate new disparity image message
    stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
    disp_msg->header         = l_info_msg->header;
    disp_msg->image.header   = l_info_msg->header;

    // Compute window of (potentially) valid disparities
    int border   = block_matcher_.getCorrelationWindowSize() / 2;
    int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
    int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
    int right  = disp_msg->image.width - 1 - wtf;
    int top    = border;
    int bottom = disp_msg->image.height - 1 - border;
    disp_msg->valid_window.x_offset = left;
    disp_msg->valid_window.y_offset = top;
    disp_msg->valid_window.width    = right - left;
    disp_msg->valid_window.height   = bottom - top;

    // Perform block matching to find the disparities
    block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

    // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
    double cx_l = model_.left().cx();
    double cx_r = model_.right().cx();
    if (cx_l != cx_r) {
        cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                reinterpret_cast<float*>(&disp_msg->image.data[0]),
                disp_msg->image.step);
        cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
    }

    pub_disparity_.publish(disp_msg);
}

void DisparityNodelet::configCb(Config &config, uint32_t level)
{
    // Tweak all settings to be valid
    config.prefilter_size |= 0x1; // must be odd
    config.correlation_window_size |= 0x1; // must be odd
    config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

    // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
    // concurrently, so this is thread-safe.
    block_matcher_.setPreFilterSize(config.prefilter_size);
    block_matcher_.setPreFilterCap(config.prefilter_cap);
    block_matcher_.setCorrelationWindowSize(config.correlation_window_size);
    block_matcher_.setMinDisparity(config.min_disparity);
    block_matcher_.setDisparityRange(config.disparity_range);
    block_matcher_.setUniquenessRatio(config.uniqueness_ratio);
    block_matcher_.setTextureThreshold(config.texture_threshold);
    block_matcher_.setSpeckleSize(config.speckle_size);
    block_matcher_.setSpeckleRange(config.speckle_range);
}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::DisparityNodelet,nodelet::Nodelet)

PLUGINLIB_EXPORT_CLASS( gpuimageproc::Stereoproc, nodelet::Nodelet)
}
