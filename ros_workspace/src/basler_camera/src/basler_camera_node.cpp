#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <basler_camera/basler_node.h>

#include <sys/types.h>
#include <signal.h>

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
BaslerNode::service_enable_publish(platform_motion_msgs::Enable::Request &req, platform_motion_msgs::Enable::Response &resp)
{
  publish_enabled = req.state;
  resp.state = req.state;
  return true;
}

void
BaslerNode::topic_enable_publish(std_msgs::BoolConstPtr msg)
{
    publish_enabled = msg->data;
}


bool
BaslerNode::service_enable_grab(platform_motion_msgs::Enable::Request &req, platform_motion_msgs::Enable::Response &resp)
{
  resp.state = do_enable_grab(req.state);
  return true;
}

void
BaslerNode::topic_enable_grab(std_msgs::BoolConstPtr msg)
{
    do_enable_grab(msg->data);
}

bool
BaslerNode::do_enable_grab(bool state)
{
  if(state && !camera.IsGrabbing())
  {
      try
      {
          camera.StartGrabbing( Pylon::GrabStrategy_LatestImageOnly, Pylon::GrabLoop_ProvidedByInstantCamera);
          ROS_INFO_STREAM("Started grabbing.");
      }
      catch(Pylon::RuntimeException &e)
      {
          ROS_ERROR_STREAM("Unable to start grabbing: " << e.GetDescription());
          shutdown();
          ros::shutdown();
      }
  }
  else if(!state && camera.IsGrabbing())
  {
      try
      {
          camera.StopGrabbing();
          ROS_INFO_STREAM("Stopped grabbing.");
      }
      catch(Pylon::RuntimeException &e)
      {
          ROS_ERROR_STREAM("Unable to stop grabbing: " << e.GetDescription());
          shutdown();
          ros::shutdown();
      }
  }
  grab_enabled = camera.IsGrabbing();
  return grab_enabled;
}

void
BaslerNode::OnImageGrabbed( Pylon::CInstantCamera& unused_camera, const Pylon::CGrabResultPtr& ptrGrabResult)
{
    (void)unused_camera;
    sensor_msgs::Image img_msg;
    ros::Time timestamp = ros::Time::now();
    if (ptrGrabResult->GrabSucceeded())
    {
        if(publish_enabled)
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
    grab_enabled(nh.param("start_grabbing", true)),
    publish_enabled(nh.param("start_enabled", false))
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
    server.setCallback(boost::bind(&BaslerNode::configure_callback, this, _1, _2));

    // This should be a param!
    converter_.OutputPixelFormat = Pylon::PixelType_RGB8packed;
    output_encoding = "rgb8";
    output_bytes_per_pixel = 3;

    grab_service = nh.advertiseService("enable_grabbing", &BaslerNode::service_enable_grab, this);
    grab_topic = nh.subscribe("enable_grabbing", 1, &BaslerNode::topic_enable_grab, this);

    enable_service = nh.advertiseService("enable_publish", &BaslerNode::service_enable_grab, this);
    enable_topic = nh.subscribe("enable_publish", 1, &BaslerNode::topic_enable_publish, this);

    do_enable_grab(grab_enabled);
}

void
BaslerNode::shutdown(void)
{
    if(camera.IsPylonDeviceAttached())
    {
        ROS_INFO("Attached, beginning shutdown");
        if(camera.IsGrabbing())
        {
            ROS_INFO("Stopping grab");
            camera.StopGrabbing();
        }
        if(camera.IsOpen())
        {
            ROS_INFO("closing");
            camera.Close();
        }
        ROS_INFO("detach");
        camera.DetachDevice();
        ROS_INFO("destroy");
        camera.DestroyDevice();
    }
    else
    {
        ROS_INFO("Not attached, no shutdown needed");
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
            ROS_INFO_STREAM("Empty serial number, using first device");
        } else {
            // Look up the camera by its serial number
            for (size_t i=0; i<devices.size(); i++) {
                ROS_INFO_STREAM("Checking '" << \
                        devices[i].GetSerialNumber() << \
                        "' == '" <<\
                        serial_number << "'");
                if (devices[i].GetSerialNumber() == serial_number.c_str()) {
                    camera.Attach(tlFactory.CreateDevice(devices[i]));
                    break;
                }
            }
            if(!camera.IsPylonDeviceAttached())
            {
                throw RUNTIME_EXCEPTION("No camera with specified serial number found");
            }
        }

        ROS_INFO_STREAM("using device " << camera.GetDeviceInfo().GetModelName() << \
               " serial number " << camera.GetDeviceInfo().GetSerialNumber() );
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

    try
    {
        BaslerCamera::BaslerNode node(nh);

        ros::spin();

        node.shutdown();
    }
    catch(std::exception &e)
    {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
    }

}
