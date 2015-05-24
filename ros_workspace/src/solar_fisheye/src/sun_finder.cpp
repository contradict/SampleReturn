#include "solar_fisheye/sun_finder.h"

#include <math.h>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "solpos00.h"

namespace solar_fisheye {

static double degrees(double radians) {
    return radians*180.0/M_PI;
}
static double radians(double degrees) {
    return degrees/180.0*M_PI;
}

// convert a lat/lon/time to a Vec3d in local NED co-ords
void compute_sunvec(double lat, double lon, ros::Time t, cv::Point3d & sunvec)
{
    struct posdata pdat;
    long retval;

    S_init(&pdat);
    pdat.latitude = lat;
    pdat.longitude = lon;

    //convert ros time to ymd-hms
    time_t utctime = t.toSec();
    struct tm * ymdhms = gmtime(&utctime);
    pdat.year = 1900+ymdhms->tm_year;
    pdat.daynum = 1+ymdhms->tm_yday;
    pdat.hour = ymdhms->tm_hour;
    pdat.minute = ymdhms->tm_min;
    pdat.second = ymdhms->tm_sec;
    pdat.timezone = 0; //GMT/UTC

    pdat.aspect = 0.0; //north up
    pdat.tilt = 0.0;

    pdat.function |= S_SOLAZM | S_REFRAC;

    retval = S_solpos (&pdat);  /* S_solpos function call */
    S_decode(retval, &pdat);    /* ALWAYS look at the return code! */

    if (retval == 0) {
        double zen = radians(pdat.zenref); /* 0 = up */
        double azi = radians(pdat.azim); /* N=0 E=90 S=180 W=270 */
        zen = M_PI - zen; //convert to NED */
        sunvec.z = std::cos(zen);
        sunvec.x = std::cos(azi)*std::sin(zen);
        sunvec.y = std::sin(azi)*std::sin(zen);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// taken from cv::fisheye::undistortPoints
/// output is X->right, Y->down, Z->out the lens

void unprojectPointsFisheye( cv::InputArray distorted, cv::OutputArray undistorted, cv::InputArray K, cv::InputArray D, cv::InputArray R, cv::InputArray P)
{
    // will support only 2-channel data now for points
    CV_Assert(distorted.type() == CV_32FC2 || distorted.type() == CV_64FC2);
    undistorted.create(distorted.size(), CV_MAKETYPE(distorted.depth(), 3));

    CV_Assert(P.empty() || P.size() == cv::Size(3, 3) || P.size() == cv::Size(4, 3));
    CV_Assert(R.empty() || R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(D.total() == 4 && K.size() == cv::Size(3, 3) && (K.depth() == CV_32F || K.depth() == CV_64F));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        cv::Matx33f camMat = K.getMat();
        f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        cv::Matx33d camMat = K.getMat();
        f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
    }

    cv::Vec4d k = D.depth() == CV_32F ? (cv::Vec4d)*D.getMat().ptr<cv::Vec4f>(): *D.getMat().ptr<cv::Vec4d>();

    cv::Matx33d RR = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == cv::Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    if(!P.empty())
    {
        cv::Matx33d PP;
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
        RR = PP * RR;
    }

    // start undistorting
    const cv::Vec2f* srcf = distorted.getMat().ptr<cv::Vec2f>();
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec3f* dstf = undistorted.getMat().ptr<cv::Vec3f>();
    cv::Vec3d* dstd = undistorted.getMat().ptr<cv::Vec3d>();

    size_t n = distorted.total();
    int sdepth = distorted.depth();

    for(size_t i = 0; i < n; i++ )
    {
        cv::Vec2d pi = sdepth == CV_32F ? (cv::Vec2d)srcf[i] : srcd[i];  // image point
        cv::Vec2d pw((pi[0] - c[0])/f[0], (pi[1] - c[1])/f[1]);      // world point

        double theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);
        double theta = theta_d;
        if (theta_d > 1e-8)
        {
            // compensate distortion iteratively
            for(int j = 0; j < 10; j++ )
            {
                double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
                theta = theta_d / (1 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
            }
        }
        double z = std::cos(theta);
        double r = std::sin(theta);

        cv::Vec3d pu = cv::Vec3d(r*pw[0], r*pw[1], z); //undistorted point

        // reproject
        cv::Vec3d pr = RR * pu; // rotated point optionally multiplied by new camera matrix
        cv::Vec3d fi;       // final
        normalize(pr, fi);

        if( sdepth == CV_32F )
            dstf[i] = fi;
        else
            dstd[i] = fi;
    }
}

SunFinder::SunFinder()
{
    ros::NodeHandle nh;
    std::string camera_ns = nh.resolveName("camera");
    //ROS_INFO_STREAM("camera_ns: " << camera_ns);

    it_ = new image_transport::ImageTransport(nh);
    sub_ = it_->subscribeCamera(camera_ns + "/image_raw", 0, &SunFinder::imageCallback, this);
    meas_pub_ = nh.advertise<solar_fisheye::SunSensor>("measurement", 3);

    reconfigure_server_.setCallback(boost::bind(&SunFinder::configure, this, _1, _2));
}

void
SunFinder::configure(SolarFisheyeConfig &config, uint32_t level)
{
    (void)level;
    config_ = config;
}

void
SunFinder::imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
        const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    ROS_DEBUG("Got an image");
    cv::Mat imageMat = cv_bridge::toCvShare(image_msg, "mono8")->image;
    model_.fromCameraInfo(info_msg);

    // do cool stuff with imageMat and info_msg
    cv::Mat img_thr = cv::Mat::zeros( imageMat.size(), CV_8UC1 );
    cv::threshold(imageMat, img_thr, config_.noise_threshold, 255, cv::THRESH_TOZERO);

    /// Get the moments
    cv::Moments mu = cv::moments(img_thr, false );
    ///  Get the mass center:
    cv::Point2f mc;
    if (mu.m00 != 0) {
        mc.x = static_cast<float>(mu.m10/mu.m00);
        mc.y = static_cast<float>(mu.m01/mu.m00);
    } else {
        mc.x = 0;
        mc.y = 0;
    }
    //clamp centroid to be on image
    mc.x = std::min(float(imageMat.size().width), std::max(mc.x, 0.0f));
    mc.y = std::min(float(imageMat.size().height), std::max(mc.y, 0.0f));

    if ( (img_thr.at<uint8_t>(mc) >= config_.min_centroid)
            && (sqrt(mu.mu20) <= config_.max_dev)
            //&& (sqrt(mu.mu11) <= config_.max_dev)
            && (sqrt(mu.mu02) <= config_.max_dev)
    ) {
        std::vector<cv::Point2f> pts;
        pts.push_back(mc);
        std::vector<cv::Point3f> undist_pts;

        unprojectPointsFisheye(pts, undist_pts, model_.fullIntrinsicMatrix(), model_.distortionCoeffs(), cv::Mat(), cv::Mat());
        solar_fisheye::SunSensor meas;
        meas.header = info_msg->header;
        meas.measurement.x = undist_pts[0].x;
        meas.measurement.y = undist_pts[0].y;
        meas.measurement.z = undist_pts[0].z;

        cv::Point3d sun;
        compute_sunvec(config_.lat, config_.lon, image_msg->header.stamp, sun);
        meas.reference.x = sun.x;
        meas.reference.y = sun.y;
        meas.reference.z = sun.z;
        meas_pub_.publish(meas);
    }
}

}
