#pragma once

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/MagneticField.h>

#include <thread>

#include "mip_sdk.h"

class MicroStrainDriver 
{
    public:
        MicroStrainDriver(ros::NodeHandle &h, mip_interface *device_interface);
        ~MicroStrainDriver();
        bool Init();
        bool spin();
        bool SetIdle();
        bool ResumeOutput();
        bool SetupAHRS();
        bool SetupGPS();
        bool SetAHRSContinuous();
        bool SetGPSContinuous();
        void ProcessAHRSPacket(u8 *packet, u16 packet_size, u8 callback_type);
        void ProcessGPSPacket(u8 *packet, u16 packet_size, u8 callback_type);

    private:
        mip_interface *device_interface_;
        ros::NodeHandle node_handle_;
        ros::NodeHandle nh_priv_;
       
        // Publisher
        ros::Publisher imu_pub_;
        ros::Publisher gps_pub_;
        ros::Publisher magnetometer_pub_;

        // properties
        int baud_rate_;
        bool m_bGetGPS;
        bool m_bGetAHRS;

        bool m_bGetTimeStampPpsAHRS;
        bool m_bGetTimeStampGpsCorrelationAHRS;
        bool m_bGetEulerAHRS;
        bool m_bGetQuaternionAHRS;
        bool m_bGetAccelerometerAHRS;
        bool m_bGetGyroAHRS;
        bool m_bGetMagnetometerAHRS;
        int  m_nHzGPS;
        int  m_nHzAHRS;
        
        // imu covariances etc
        double angular_velocity_stdev_, angular_velocity_covariance_;
        double linear_acceleration_covariance_, linear_acceleration_stddev_;
        double orientation_covariance_, orientation_stdev_;

        // when to publish
        bool got_linear_acceleration_;
        bool got_angular_velocity_;
        bool got_orientation_;

        bool got_gps_fix_;
        bool got_gps_status_;

        double linear_acceleration_stdev_;

        // ROS stuff
        bool publish_imu_;
        bool publish_gps_;
        std::string imu_frame_id_;
        std::string gps_frame_id_;

        sensor_msgs::Imu imu_data_; // data trickles in over 3 packets
        sensor_msgs::NavSatFix gps_fix_data_; // data trickles in over 3 packets
        sensor_msgs::NavSatStatus status_data_; // data trickles in over 3 packets

};
