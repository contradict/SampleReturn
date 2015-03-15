#include <microstrain_imu_driver/MicroStrainDriver.h>
#include "mip_sdk.h"
#include "mip_sdk_interface.h"
#include "mip_gx3_35.h"

MicroStrainDriver *pDriver;

void AHRSCallbackFunc(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
    // Process packet
    pDriver->ProcessAHRSPacket(packet, packet_size, callback_type);
}

void GPSCallbackFunc(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
    // Process packet
    pDriver->ProcessGPSPacket(packet, packet_size, callback_type);
}


void ThreadCaptureFunc(mip_interface *device_interface)
{
    while(!ros::isShuttingDown())
    {
        mip_interface_update(device_interface,true);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "microstrain_3DMGX3_node");
    
    ros::NodeHandle nh;

    mip_interface* device_interface = new mip_interface;
    pDriver = new MicroStrainDriver(nh, device_interface);

    bool use_gps, use_ahrs;

    nh.param<bool>("gps_on", use_gps, true);
    nh.param<bool>("ahrs_on", use_ahrs, true);

    if(!pDriver->Init())
    {
        ROS_INFO("Initialization failed");
        exit(0);
    }
    else
        ROS_INFO("Node up and running");

    if(use_ahrs)
    {
        pDriver->SetupAHRS();
    
        //Setup callbacks for AHRS mode
        if(mip_interface_add_descriptor_set_callback(device_interface,MIP_AHRS_DATA_SET,NULL,   &AHRSCallbackFunc) != MIP_INTERFACE_OK)
        {
            ROS_INFO("Error registering callback");
            return false;
        }
        pDriver->SetAHRSContinuous();
    }

    if(use_gps)
    {
        pDriver->SetupGPS();
    
        //Setup callbacks for GPS mode
        if(mip_interface_add_descriptor_set_callback(device_interface,MIP_GPS_DATA_SET,NULL, &GPSCallbackFunc) != MIP_INTERFACE_OK)
        {
            ROS_INFO("Error registering callback");
            return false;
        }
        pDriver->SetGPSContinuous();
    }
    
    std::thread capture(ThreadCaptureFunc, device_interface);

    while(!ros::isShuttingDown())
    {
        ros::spin();
    }

    capture.join();
    delete(pDriver);
    delete(device_interface);
}

