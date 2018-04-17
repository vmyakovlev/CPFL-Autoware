/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>

#include <bgapi2_genicam.hpp>

class BaumerVCXU24
{
public:
  BaumerVCXU24();
  ~BaumerVCXU24();
  void spin();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  image_transport::ImageTransport *image_transport_;
  std::vector<image_transport::Publisher> image_pubs_;
  // baumer
  BGAPI2::System *b_system_;
  BGAPI2::Interface *b_interface_;
  BGAPI2::DeviceList *b_device_list_;
  // configrations
  double exposure_time_;  // [ms]
};

BaumerVCXU24::BaumerVCXU24() : nh_(), private_nh_("~")
{
  // get rosparam
  private_nh_.param("exposure_time", exposure_time_, 10.0); // [ms]

  try
  {
    // initialize baumer system
    BGAPI2::SystemList *b_system_list = BGAPI2::SystemList::GetInstance();
    b_system_list->Refresh();
    b_system_ = b_system_list->begin()++->second;  // using usb3 type
    b_system_->Open();

    // initialize baumer interface
    BGAPI2::InterfaceList *b_interface_list = b_system_->GetInterfaces();
    b_interface_list->Refresh(100);
    b_interface_ = b_interface_list->begin()->second; // using 1st interface
    b_interface_->Open();

    // get baumer device list
    b_device_list_ = b_interface_->GetDevices();
    b_device_list_->Refresh(100);

    // initialize ros image_transport
    image_transport_ = new image_transport::ImageTransport(nh_);

    size_t camera_idx = 0;
    for (auto itr = b_device_list_->begin(); itr != b_device_list_->end(); ++itr)
    {
      // open device
      BGAPI2::Device *b_device = itr->second;
      b_device->Open();

      ROS_INFO_STREAM("ID: " << b_device->GetID());
      ROS_INFO_STREAM("Vendor: " << b_device->GetVendor());
      ROS_INFO_STREAM("Model: " << b_device->GetModel());
      ROS_INFO_STREAM("TL Type: " << b_device->GetTLType());
      ROS_INFO_STREAM("Display Name: " << b_device->GetDisplayName());
      ROS_INFO_STREAM("Serial Number: " << b_device->GetSerialNumber());

      // set camera configurations
      b_device->GetRemoteNode("TriggerMode")->SetString("Off");
      BGAPI2::String sExposureNodeName = "";
      if (b_device->GetRemoteNodeList()->GetNodePresent("ExposureTime"))
        sExposureNodeName = "ExposureTime";
      else if (b_device->GetRemoteNodeList()->GetNodePresent("ExposureTimeAbs"))
        sExposureNodeName = "ExposureTimeAbs";
      b_device->GetRemoteNode(sExposureNodeName)->SetDouble(1000*exposure_time_);

      // get and open data stream
      BGAPI2::DataStreamList *b_data_stream_list = b_device->GetDataStreams();
      b_data_stream_list->Refresh();
      BGAPI2::DataStream *b_data_stream = b_data_stream_list->begin()->second;
      b_data_stream->Open();

      // get and create buffers
      BGAPI2::BufferList *b_buffer_list = b_data_stream->GetBufferList();
      BGAPI2::Buffer *b_buffer = NULL;
      for (size_t j = 0; j < 4; j++)
      {
        b_buffer = new BGAPI2::Buffer();
        b_buffer_list->Add(b_buffer);
        b_buffer->QueueBuffer();
      }

      // start camera
      b_data_stream->StartAcquisitionContinuous();
      b_device->GetRemoteNode("AcquisitionStart")->Execute();

      // create publisher
      std::string topic(std::string("image_raw"));
  		if (b_device_list_->size() > 1)
  			topic = "camera" + std::to_string(camera_idx) + "/" + topic;
      image_pubs_.push_back(image_transport_->advertise(topic, 10));
      ROS_INFO_STREAM("Publish -> " << topic);
      camera_idx++;
    }
  }
  catch (BGAPI2::Exceptions::IException& ex)
  {
    ROS_ERROR_STREAM("Error in function: " << ex.GetFunctionName());
    ROS_ERROR_STREAM("Error description: " << ex.GetErrorDescription());
    BGAPI2::SystemList::ReleaseInstance();
  }
}

BaumerVCXU24::~BaumerVCXU24()
{
  for (auto itr = b_device_list_->begin(); itr != b_device_list_->end(); ++itr)
  {
    // stop camera
    BGAPI2::Device *b_device = itr->second;
    b_device->GetRemoteNode("AcquisitionAbort")->Execute();
    b_device->GetRemoteNode("AcquisitionStop")->Execute();
    BGAPI2::DataStreamList *b_data_stream_list = b_device->GetDataStreams();
    BGAPI2::DataStream *b_data_stream = b_data_stream_list->begin()->second;
    b_data_stream->StopAcquisition();

    // clear buffers
    BGAPI2::BufferList *b_buffer_list = b_data_stream->GetBufferList();
    b_buffer_list->DiscardAllBuffers();
    while (b_buffer_list->size() > 0)
    {
      BGAPI2::Buffer *b_buffer = b_buffer_list->begin()->second;
      b_buffer_list->RevokeBuffer(b_buffer);
      delete b_buffer;
    }
    // close data stream and device
    b_data_stream->Close();
    b_device->Close();
  }

  // close interface and system
  b_interface_->Close();
  b_system_->Close();

  // release
  BGAPI2::SystemList::ReleaseInstance();
}


void BaumerVCXU24::spin()
{
  BGAPI2::Buffer *b_buffer = NULL;
  BGAPI2::Image  *b_image = NULL, *b_image_bgr = NULL;
  BGAPI2::ImageProcessor *b_image_processor = BGAPI2::ImageProcessor::GetInstance();

  while (ros::ok())
  {
    size_t camera_idx = 0;  // TODO -> multi-threading
    for (auto itr = b_device_list_->begin(); itr != b_device_list_->end(); ++itr)
    {
      // get device and filled buffer
      BGAPI2::Device *b_device = itr->second;
      BGAPI2::DataStream *b_data_stream = b_device->GetDataStreams()->begin()->second;;
      b_buffer = b_data_stream->GetFilledBuffer(1000);

      // check time out
      if (b_buffer == NULL)
      {
        ROS_ERROR("Buffer timeout after 1000 msec");
      }
      // check integrity of buffer data
      else if (b_buffer->GetIsIncomplete() == true)
      {
        ROS_ERROR("Buffer image is incomplete");
        b_buffer->QueueBuffer();
      }
      else
      {
        // get receive time
        ros::Time receive_time = ros::Time::now();

        // create image and convert to BGR8 for opencv
        b_image = b_image_processor->CreateImage(
          (bo_uint)b_buffer->GetWidth(),
          (bo_uint)(int)b_buffer->GetHeight(),
          b_buffer->GetPixelFormat(),
          b_buffer->GetMemPtr(),
          b_buffer->GetMemSize());
        b_image->TransformImage("BGR8", &b_image_bgr);

        ROS_DEBUG_STREAM("ID: " << b_device->GetID()
          << ", Frame: " << b_buffer->GetFrameID()
          << ", Pixel Format: " << b_image_bgr->GetPixelformat()
          << ", Width: " << b_image_bgr->GetWidth()
          << ", Height: " << b_image_bgr->GetHeight());

        // create opencv mat
        cv::Mat mat(cv::Size(b_image_bgr->GetWidth(), b_image_bgr->GetHeight()), CV_8UC3, b_image_bgr->GetBuffer());

        // create and publish ros message
        std_msgs::Header header;
        header.stamp = receive_time;
        header.frame_id = "camera";
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", mat).toImageMsg();
        image_pubs_[camera_idx].publish(msg);
        camera_idx++;

        b_image->Release();
        b_image_bgr->Release();
        b_buffer->QueueBuffer();
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baumer_vcxu24");
  BaumerVCXU24 node;
  node.spin();
  return 0;
}
