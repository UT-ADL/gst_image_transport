#pragma once

#include <image_transport/simple_subscriber_plugin.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <pluginlib/class_list_macros.hpp>

#include "decoders/GstDecoder.h"

class ParentSubscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
protected:
  virtual GstDecoder* get_decoder() = 0;
  void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size, const Callback& callback,
                     const ros::VoidPtr& tracked_object,
                     const image_transport::TransportHints& transport_hints) override;
  void internalCallback(const sensor_msgs::CompressedImage::ConstPtr& message, const Callback& user_cb) override;

private:
  sensor_msgs::ImagePtr out_ = boost::make_shared<sensor_msgs::Image>();
};
