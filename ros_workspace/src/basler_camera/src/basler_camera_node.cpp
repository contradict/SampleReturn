#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <basler_camera/basler_node.h>

#include <XmlRpcValue.h>

namespace BaslerCamera
{

void
BaslerNode::handle_basler_parameter(std::string name, bool value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting boolean param " << name << " to " << value << ".");
        GenApi::CBooleanPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            return;
        }
        this_node->SetValue(value);
    }
    catch (const GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void
BaslerNode::handle_basler_parameter(std::string name, int value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting int param " << name << " to " << value << ".");
        GenApi::CIntegerPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            return;
        }
        this_node->SetValue(value);
    }
    catch (const GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void
BaslerNode::handle_basler_parameter(std::string name, double value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting float param " << name << " to " << value << ".");
        GenApi::CFloatPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            return;
        }
        this_node->SetValue(value);
    }
    catch (const GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void
BaslerNode::handle_basler_parameter(std::string name, std::string value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting enum param " << name << " to " << value << ".");
        GenApi::CEnumerationPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            return;
        }
        if (!IsAvailable(this_node->GetEntryByName(value.c_str())))
        {
            ROS_ERROR_STREAM("Valuer '" << value << "' isn't available for basler param '" << name << "'.");
            return;
        }
        this_node->FromString(value.c_str());
    }
    catch (const GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void
BaslerNode::handle_basler_parameter(XmlRpc::XmlRpcValue& param)
{
  ros::NodeHandle nh("~");
  std::string type = param["type"];
  if ("boolean" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeBoolean,
                   "Type of value for %s must be boolean", std::string(param["name"]).c_str());
    handle_basler_parameter(param["name"], (bool)param["value"]);
    nh.setParam(param["name"], (bool)param["value"]);
  }
  else if ("int" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeInt,
                   "Type of value for %s must be int", std::string(param["name"]).c_str());
    handle_basler_parameter(param["name"], (int)param["value"]);
    nh.setParam(param["name"], (int)param["value"]);
  }
  else if ("float" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble,
                   "Type of value for %s must be float", std::string(param["name"]).c_str());
    handle_basler_parameter(param["name"], (double)param["value"]);
    nh.setParam(param["name"], (double)param["value"]);
  }
  else if ("enum" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeString,
                   "Type of value for %s must be string", std::string(param["name"]).c_str());
    handle_basler_parameter(param["name"], (std::string)param["value"]);
    nh.setParam(param["name"], (std::string)param["value"]);
  }
  else
  {
    ROS_FATAL_STREAM("Unknown param type for parameter " << param["name"] << ": " << type);
  }
}

void
BaslerNode::configure_callback(basler_camera::CameraConfig &config, uint32_t level)
{
    (void) level;
    for (std::vector<basler_camera::CameraConfig::AbstractParamDescriptionConstPtr>::const_iterator _i = config.__getParamDescriptions__().begin(); _i != config.__getParamDescriptions__().end(); ++_i)
    {
        boost::any val;
        (*_i)->getValue(config, val);
        if("bool" == (*_i)->type)
        {
            handle_basler_parameter((*_i)->name,  boost::any_cast<bool>(val));
        }
        else if("double" == (*_i)->type)
        {
            handle_basler_parameter((*_i)->name,  boost::any_cast<double>(val));
        }
        else if("int" == (*_i)->type)
        {
            handle_basler_parameter((*_i)->name,  boost::any_cast<int>(val));
        }
        else if("str" == (*_i)->type)
        {
            handle_basler_parameter((*_i)->name,  boost::any_cast<std::string>(val));
        }
        else
        {
            ROS_FATAL_STREAM("Unknown param type for config parameter " << (*_i)->name << ": " << (*_i)->type);
        }
    }
}

bool
BaslerNode::service_enable(platform_motion_msgs::Enable::Request &req, platform_motion_msgs::Enable::Response &resp)
{
  enabled = req.state;
  if(enabled && !camera.IsGrabbing())
  {
      camera.StartGrabbing( Pylon::GrabStrategy_LatestImageOnly, Pylon::GrabLoop_ProvidedByInstantCamera);
  }
  else if(!enabled && camera.IsGrabbing())
  {
      camera.StopGrabbing();
  }
  resp.state = req.state;
  return true;  
}


void
BaslerNode::OnImageGrabbed( Pylon::CInstantCamera& unused_camera, const Pylon::CGrabResultPtr& ptrGrabResult)
{
    (void)unused_camera;
    sensor_msgs::Image img_msg;
    ros::Time timestamp = ros::Time::now();
    if (ptrGrabResult->GrabSucceeded())
    {
        converter_.Convert(pylon_image_, ptrGrabResult);
        img_msg.header.stamp = timestamp;
        img_msg.header.frame_id = frame_id;
        sensor_msgs::fillImage(img_msg,
                output_encoding,
                ptrGrabResult->GetHeight(),
                ptrGrabResult->GetWidth(),
                output_bytes_per_pixel*ptrGrabResult->GetWidth(),
                pylon_image_.GetBuffer());
        sensor_msgs::CameraInfo info = cinfo_manager_->getCameraInfo();
        info.header.stamp = img_msg.header.stamp;
        info.header.frame_id = img_msg.header.frame_id;
        cam_pub_.publish(img_msg, info);
    }
    else
    {
        ROS_ERROR_STREAM("Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription());
    }
}

BaslerNode::BaslerNode(ros::NodeHandle &nh) :
    camera_info_url(nh.param("camera_info_url", std::string(""))),
    frame_id(nh.param("frame_id", std::string("camera"))),
    serial_number(nh.param("serial_number", std::string(""))),
    camera_name(nh.getNamespace()),
    enabled(nh.param("start_enabled", false))
{

    if(nh.hasParam("frame_rate"))
    {
        ROS_ERROR("frame_rate param has been removed. Please use AcquisitionFrameRate and remember to set AcquisitionFrameRateEnable=True");
        nh.deleteParam("frame_rate");
    }

    find_camera();

    camera.Open();

    cinfo_manager_ = new camera_info_manager::CameraInfoManager(nh, camera_name, camera_info_url);
    it_ = new image_transport::ImageTransport(nh);
    cam_pub_ = it_->advertiseCamera("image", 1);

    camera.RegisterImageEventHandler(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None);

    camera.RegisterConfiguration(new Pylon::CAcquireContinuousConfiguration , Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);

    // use dynamic reconfigure to trigger a camera configuration
    dynamic_reconfigure::Server<basler_camera::CameraConfig> server;
    server.setCallback(boost::bind(&BaslerNode::configure_callback, this, _1, _2));

    // This should be a param!
    converter_.OutputPixelFormat = Pylon::PixelType_RGB8packed;
    output_encoding = "rgb8";
    output_bytes_per_pixel = 3;

    enable_service = nh.advertiseService("enable_publish", &BaslerNode::service_enable, this);

    if(enabled)
        camera.StartGrabbing( Pylon::GrabStrategy_LatestImageOnly, Pylon::GrabLoop_ProvidedByInstantCamera);

}

    void
BaslerNode::shutdown(void)
{
    if(camera.IsPylonDeviceAttached())
    {
        if(camera.IsGrabbing())
            camera.StopGrabbing();
        if(camera.IsOpen())
            camera.Close();
    }
}

void
BaslerNode::find_camera()
{
    try
    {
        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices(devices) == 0)
        {
            throw RUNTIME_EXCEPTION("No camera present.");
        }

        if (serial_number == "") {
            // Create an instant camera object for the camera device found first.
            camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        } else {
            // Look up the camera by its serial number
            for (size_t i=0; i<devices.size(); i++) {
                if (devices[i].GetSerialNumber().c_str() == serial_number) {
                    camera.Attach(tlFactory.CreateDevice(devices[i]));
                    break;
                }
            }
            if(!camera.IsPylonDeviceAttached())
            {
                throw RUNTIME_EXCEPTION("No camera with specified serial number found");
            }
        }

        ROS_INFO_STREAM("using device " << camera.GetDeviceInfo().GetModelName());
    }

    catch (GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM ("An exception occurred during setup: " << e.GetDescription());
        throw;
    }
}

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "basler_camera");
    ros::NodeHandle nh("~");

    BaslerCamera::BaslerNode node(nh);

    ros::spin();

    node.shutdown();
}
