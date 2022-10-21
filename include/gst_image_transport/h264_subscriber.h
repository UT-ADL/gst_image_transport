#pragma once

#include <image_transport/simple_subscriber_plugin.h>
#include <image_transport/transport_hints.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <pluginlib/class_list_macros.hpp>

#include "decoders/GstDecoder.h"

namespace h264_image_transport
{

class H264Subscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
  H264Subscriber() = default;
  virtual ~H264Subscriber() = default;

  virtual std::string getTransportName() const
  {
    return TRANSPORT_NAME;
  }

protected:
  virtual void internalCallback(const sensor_msgs::CompressedImage::ConstPtr& message, const Callback& user_cb);

  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const image_transport::TransportHints& transport_hints);

private:
  const std::string TRANSPORT_NAME = "h264";
  sensor_msgs::ImagePtr out_ = boost::make_shared<sensor_msgs::Image>();
  std::unique_ptr<GstDecoder> decoder_ = std::make_unique<GstDecoder>(TRANSPORT_NAME);
};

};  // namespace h264_image_transport