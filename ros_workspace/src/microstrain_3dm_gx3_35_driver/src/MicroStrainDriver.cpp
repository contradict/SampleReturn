#include <iostream>

#include <tf/transform_datatypes.h>
#include <microstrain_imu_driver/MicroStrainDriver.h>

#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "mip_sdk.h"
#include "mip_sdk_gps.h"
#include "mip_gx3_35.h"
#include "byteswap_utilities.h"
#pragma GCC diagnostic pop


//IMUDriverDataCallback   MicroStrainDriver::mIMUCallback = nullptr;
//PosysDriverDataCallback MicroStrainDriver::mPosysCallback = nullptr;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;
const double GRAVITY_MAGNITUDE = 9.80665;

#define _LINUX

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <time.h>
void Sleep(unsigned int time)
{
    struct timespec t,r;
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;
    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

///////////////////////////////////////////////////////////////////////////////
MicroStrainDriver::MicroStrainDriver(ros::NodeHandle &h, mip_interface *device_interface)
    : nh_priv_("~"), device_interface_(device_interface)
{
    // Parameters
    ros::param::param<int>("~baud_rate",baud_rate_,115200);
    ros::param::param<double>("~linear_acceleration_stdev",linear_acceleration_stdev_,0.098);
    ros::param::param<double>("~angular_velocity_stdev",angular_velocity_stdev_,0.012);
    ros::param::param<double>("~orientation_stdev",orientation_stdev_,0.035);
    ros::param::param<std::string>("~imu_frame",imu_frame_id_,"imu");
    ros::param::param<std::string>("~gps_frame",gps_frame_id_,"gps");

    imu_pub_ = h.advertise<sensor_msgs::Imu>("imu",100);
    gps_pub_ = h.advertise<sensor_msgs::NavSatFix>("gps",10);
    magnetometer_pub_ = h.advertise<sensor_msgs::MagneticField>("magnetometer",100);

    // TODO(jmf): Actually fill this out from constructor

    // TODO(jmf): Do a Posys driver out of these
    m_bGetGPS  = false;
    m_nHzGPS   = 1;
    m_bGetEulerAHRS			= false;
    m_bGetQuaternionAHRS	= true;

    m_bGetAHRS = true;
    m_nHzAHRS  = 200;
    m_bGetAccelerometerAHRS = true;
    m_bGetGyroAHRS	        = true;
    m_bGetMagnetometerAHRS  = true;
    m_bGetTimeStampPpsAHRS  = true;

    double linear_acceleration_covariance = linear_acceleration_stdev_*linear_acceleration_stdev_;
    double orientation_covariance = orientation_stdev_*orientation_stdev_;
    double angular_velocity_covariance = angular_velocity_stdev_*angular_velocity_stdev_;
    
    imu_data_.linear_acceleration_covariance[0]=linear_acceleration_covariance;
    imu_data_.linear_acceleration_covariance[4]=linear_acceleration_covariance;
    imu_data_.linear_acceleration_covariance[8]=linear_acceleration_covariance;

    imu_data_.angular_velocity_covariance[0]=angular_velocity_covariance;
    imu_data_.angular_velocity_covariance[4]=angular_velocity_covariance;
    imu_data_.angular_velocity_covariance[8]=angular_velocity_covariance;
    
    imu_data_.orientation_covariance[0]=orientation_covariance;
    imu_data_.orientation_covariance[4]=orientation_covariance;
    imu_data_.orientation_covariance[8]=orientation_covariance;
}

///////////////////////////////////////////////////////////////////////////////
MicroStrainDriver::~MicroStrainDriver()
{
    if(device_interface_->port_handle)
    {
        if(m_bGetAHRS) {
            // Take out of continuous ahrs mode
            u8 ahrs_enable = 0;
            while(mip_3dm_cmd_continuous_data_stream(
                device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                MIP_3DM_AHRS_DATASTREAM, &ahrs_enable) != MIP_INTERFACE_OK) { }
        }

        if(m_bGetGPS) {
            // Take out of continuous GPS mode
            u8 gps_enable = 0;
            while(mip_3dm_cmd_continuous_data_stream(
                 device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                  MIP_3DM_GPS_DATASTREAM, &gps_enable) != MIP_INTERFACE_OK) { }
        }
    }
}

void MicroStrainDriver::ProcessAHRSPacket(u8 *packet, u16 /*packet_size*/, u8 callback_type)
{
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch(callback_type)
    {
        case MIP_INTERFACE_CALLBACK_VALID_PACKET:
        {
            // Read each field in packet.
            // For little-endian targets, byteswap the data field
            while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
            {
                switch(field_header->descriptor)
                {
                    case MIP_AHRS_DATA_TIME_STAMP_PPS:
                    {
                        mip_ahrs_1pps_timestamp curr_ahrs_pps_timestamp;
                        memcpy(&curr_ahrs_pps_timestamp, field_data, sizeof(mip_ahrs_1pps_timestamp));
                        mip_ahrs_1pps_timestamp_byteswap(&curr_ahrs_pps_timestamp);

                        // To Do: Publish a TimeReference message here
                    }
                    break;

                    // Scaled Accelerometer
                    case MIP_AHRS_DATA_ACCEL_SCALED:
                    {
                        mip_ahrs_scaled_accel curr_ahrs_accel;
                        memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));
                        mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

                        imu_data_.linear_acceleration.x = curr_ahrs_accel.scaled_accel[0];
                        imu_data_.linear_acceleration.y = curr_ahrs_accel.scaled_accel[1];
                        imu_data_.linear_acceleration.z = curr_ahrs_accel.scaled_accel[2];
                        got_linear_acceleration_=1;
                    }
                    break;

                    // Scaled Gyro
                    case MIP_AHRS_DATA_GYRO_SCALED:
                    {
                        mip_ahrs_scaled_gyro curr_ahrs_gyro;
                        memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));
                        mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

                        imu_data_.angular_velocity.x = curr_ahrs_gyro.scaled_gyro[0];
                        imu_data_.angular_velocity.y = curr_ahrs_gyro.scaled_gyro[1];
                        imu_data_.angular_velocity.z = curr_ahrs_gyro.scaled_gyro[2];
                        got_angular_velocity_=1;
                    }
                    break;

                     // Scaled Magnetometer
                    case MIP_AHRS_DATA_MAG_SCALED:
                    {
                        sensor_msgs::MagneticField magnetometer_data;
                        
                        mip_ahrs_scaled_mag curr_ahrs_mag;
                        memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));
                        mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

                        magnetometer_data.header.stamp = ros::Time::now();
                        magnetometer_data.header.frame_id = imu_frame_id_;
                        magnetometer_data.magnetic_field.x = curr_ahrs_mag.scaled_mag[0];
                        magnetometer_data.magnetic_field.y = curr_ahrs_mag.scaled_mag[1];
                        magnetometer_data.magnetic_field.z = curr_ahrs_mag.scaled_mag[2];

                        // Just publish
                        magnetometer_pub_.publish(magnetometer_data);
                    }break;

                    case MIP_AHRS_DATA_QUATERNION:
                    {
                        mip_ahrs_quaternion curr_ahrs_quat;
                        memcpy(&curr_ahrs_quat, field_data, sizeof(mip_ahrs_quaternion));
                        mip_ahrs_quaternion_byteswap(&curr_ahrs_quat);

                        imu_data_.orientation.x = curr_ahrs_quat.q[0];
                        imu_data_.orientation.y = curr_ahrs_quat.q[1];
                        imu_data_.orientation.z = curr_ahrs_quat.q[2];
                        imu_data_.orientation.w = curr_ahrs_quat.q[3];
                        got_orientation_=1;
                    }
                    break;

                    case MIP_AHRS_DATA_EULER_ANGLES:
                    {
                        mip_ahrs_euler_angles curr_ahrs_euler_angles;
                        memcpy(&curr_ahrs_euler_angles, field_data, sizeof(mip_ahrs_euler_angles));
                        mip_ahrs_euler_angles_byteswap(&curr_ahrs_euler_angles);
                        imu_data_.orientation = tf::createQuaternionMsgFromRollPitchYaw(curr_ahrs_euler_angles.roll, curr_ahrs_euler_angles.pitch, curr_ahrs_euler_angles.yaw);
                        got_orientation_=1;
                    }break;
                    
                    default: 
                    break;
                }
            }

            if(got_linear_acceleration_ && got_angular_velocity_ && got_orientation_){
                imu_data_.header.stamp = ros::Time::now();    
                imu_data_.header.frame_id = imu_frame_id_;
                imu_pub_.publish(imu_data_);
                got_linear_acceleration_=0;
                got_angular_velocity_=0;
                got_orientation_=0;
            }
        }break;

            //Handle checksum error packets
        case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
        {
            std::cerr << "Checksome error!" << std::endl;
        }break;

            //Handle timeout packets
        case MIP_INTERFACE_CALLBACK_TIMEOUT:
        {
            std::cerr << "Timeout" << std::endl;
        }break;
        default: break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// C callback function for MIP library
///////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::ProcessGPSPacket(u8 *packet, u16 /*packet_size*/, u8 callback_type)
{
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch(callback_type)
    {
        case MIP_INTERFACE_CALLBACK_VALID_PACKET:
        {
            while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
            {
                switch(field_header->descriptor)
                {
                    case MIP_GPS_DATA_FIX_INFO:
                    {
                        mip_gps_fix_info curr_gps_fix_info;
                        memcpy(&curr_gps_fix_info, field_data, sizeof(mip_gps_fix_info));
                        mip_gps_fix_info_byteswap(&curr_gps_fix_info);

                        switch(curr_gps_fix_info.fix_type)
                        {
                            case MIP_GPS_FIX_TYPE_3D:
                            {
                                status_data_.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                                break;
                            }
                            case MIP_GPS_FIX_TYPE_2D:
                            {
                                status_data_.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                                break;
                            }
                            case MIP_GPS_FIX_TYPE_TIME_ONLY:
                            {
                                status_data_.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                                break;
                            }
                            case MIP_GPS_FIX_TYPE_NONE:
                            {
                                status_data_.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                                break;
                            }
                            case MIP_GPS_FIX_TYPE_INVALID:
                            {
                                status_data_.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                                break;
                            }
                        }   
                        status_data_.service=sensor_msgs::NavSatStatus::SERVICE_GPS;
                        got_gps_status_=1;
                    }break;
                    
                    case MIP_GPS_DATA_LLH_POS:
                    {
                        mip_gps_llh_pos curr_gps_llh_pos;
                        memcpy(&curr_gps_llh_pos, field_data, sizeof(mip_gps_llh_pos));
                        mip_gps_llh_pos_byteswap(&curr_gps_llh_pos);

                        gps_fix_data_.header.stamp = ros::Time::now();
                        gps_fix_data_.header.frame_id = gps_frame_id_;
                        gps_fix_data_.latitude = curr_gps_llh_pos.latitude;
                        gps_fix_data_.longitude = curr_gps_llh_pos.longitude;
                        gps_fix_data_.altitude = curr_gps_llh_pos.msl_height;
    
                        gps_fix_data_.position_covariance[0]=curr_gps_llh_pos.horizontal_accuracy;
                        gps_fix_data_.position_covariance[5]=curr_gps_llh_pos.horizontal_accuracy;
                        gps_fix_data_.position_covariance[8]=curr_gps_llh_pos.vertical_accuracy;

                        gps_fix_data_.position_covariance_type=sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
                        got_gps_fix_=1;
 
                   }
                    break;

                    default: 
                    break;
                }
            }

            if(got_gps_fix_ && got_gps_status_){
                gps_fix_data_.header.stamp = ros::Time::now();    
                gps_fix_data_.header.frame_id = gps_frame_id_;
                gps_fix_data_.status = status_data_;
                gps_pub_.publish(gps_fix_data_);
                got_gps_status_=0;
                got_gps_fix_=0;
            }
        }break;

        ROS_ERROR("Problem in GPS packet");
            //Handle checksum error packets
        case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
        {
            std::cerr << "Checksome error!" << std::endl;
        }break;

            //Handle timeout packets
        case MIP_INTERFACE_CALLBACK_TIMEOUT:
        {
            std::cerr << "Timeout" << std::endl;
        }break;
        default: break;
    }
}

///////////////////////////////////////////////////////////////////////////////
bool MicroStrainDriver::Init()
{
    // Device communication parameters
    const u32 device_id = 0;    // mip_interface searches for microstrain devices
    //const u32 baudrate = 115200;

    // Start up device
    if(mip_interface_init(device_id, baud_rate_, device_interface_,
                          DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
        ROS_INFO("Error opening device, check permissions in /dev");
        return false;
    }

    // Listen to commands only (no data output)
    while(mip_base_cmd_idle(device_interface_) != MIP_INTERFACE_OK) { }

    // Get device and print model name
    base_device_info_field device_info;

    char model_name[BASE_DEVICE_INFO_PARAM_LENGTH*2+1] = {0};
    char model_number[BASE_DEVICE_INFO_PARAM_LENGTH*2+1] = {0};
    char serial_number[BASE_DEVICE_INFO_PARAM_LENGTH*2+1] = {0};
   
    while(mip_base_cmd_get_device_info(device_interface_, &device_info) != MIP_INTERFACE_OK) { }

    memcpy(model_name, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH*2);
    memcpy(model_number, device_info.model_number, BASE_DEVICE_INFO_PARAM_LENGTH*2);
    memcpy(serial_number, device_info.serial_number, BASE_DEVICE_INFO_PARAM_LENGTH*2);
    ROS_INFO("Connected to%s, \n Model Number %s \n Serial Number %s",model_name,  model_number, serial_number);
    return true;
}

/* Put into mode that it only listens for commands */

bool MicroStrainDriver::SetIdle()
{
    while(mip_base_cmd_idle(device_interface_) != MIP_INTERFACE_OK) { }
    return true;
}

/*After Idle mode */
bool MicroStrainDriver::ResumeOutput()
{
    while(mip_base_cmd_resume(device_interface_) != MIP_INTERFACE_OK){ }
    return true;
}

///////////////////////////////////////////////////////////////////////////////

bool MicroStrainDriver::SetupAHRS()
{
    std::cout << "Activating AHRS..." << std::endl;

    // Turn AHRS On
    u8 power_state = MIP_3DM_POWER_STATE_ON;
    while(mip_3dm_cmd_power_state(
        device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_POWER_STATE_DEVICE_AHRS, &power_state) != MIP_INTERFACE_OK) { }

    // Specify Data we're interested in receiving (and level of decimation)
    u8 ahrs_data_stream_format_num_entries;
    u8  ahrs_data_stream_format_descriptors[10] = {0};
    u16 ahrs_data_stream_format_decimation[10]  = {0};

    int idx		   = 0;
    int decimation = 1000/m_nHzAHRS;

    if(m_bGetTimeStampPpsAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_TIME_STAMP_PPS;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetTimeStampGpsCorrelationAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_TIME_STAMP_GPS;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetEulerAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_EULER_ANGLES;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetQuaternionAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_QUATERNION;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetAccelerometerAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_ACCEL_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetGyroAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_GYRO_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetMagnetometerAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_MAG_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }

    ahrs_data_stream_format_num_entries = idx;

    while(mip_3dm_cmd_ahrs_message_format(
        device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
        &ahrs_data_stream_format_num_entries, ahrs_data_stream_format_descriptors,
        ahrs_data_stream_format_decimation) != MIP_INTERFACE_OK) { }
}

bool MicroStrainDriver::SetAHRSContinuous()
{
    // Place in continuous ahrs mode
    u8 ahrs_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
        device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_AHRS_DATASTREAM, &ahrs_enable) != MIP_INTERFACE_OK) { }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
bool MicroStrainDriver::SetupGPS()
{
    std::cout << "Activating GPS..." << std::endl;
    u8 power_state = MIP_3DM_POWER_STATE_ON;

    // Turn GPS On
    while(mip_3dm_cmd_power_state(
        device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_POWER_STATE_DEVICE_GPS, &power_state) != MIP_INTERFACE_OK) { }

    // Specify Data we're interested in receiving (and level of decimation)
    u8  gps_data_stream_format_num_entries	  = 2;
    u8  gps_data_stream_format_descriptors[2] = {0};
    u16 gps_data_stream_format_decimation[2]  = {0};

    gps_data_stream_format_descriptors[0]     = MIP_GPS_DATA_LLH_POS;
    gps_data_stream_format_decimation[0]      = 4/m_nHzGPS;

    gps_data_stream_format_descriptors[1]     = MIP_GPS_DATA_FIX_INFO;
    gps_data_stream_format_decimation[1]      = 4/m_nHzGPS;
    
    while(mip_3dm_cmd_gps_message_format(
        device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
        &gps_data_stream_format_num_entries, gps_data_stream_format_descriptors,
        gps_data_stream_format_decimation) != MIP_INTERFACE_OK) { }

    return true;
}

bool MicroStrainDriver::SetGPSContinuous()
{
    // Place in continuous GPS mode
    u8 gps_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
         device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
          MIP_3DM_GPS_DATASTREAM, &gps_enable) != MIP_INTERFACE_OK) { }

    return true;
}

