#include "gst_image_transport/parent_subscriber.h"

void ParentSubscriber::subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                     const image_transport::SubscriberPlugin::Callback& callback,
                                     const ros::VoidPtr& tracked_object,
                                     const image_transport::TransportHints& transport_hints)
{
  queue_size = 10;
  SimpleSubscriberPlugin::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
}

void ParentSubscriber::internalCallback(const sensor_msgs::CompressedImage_<std::allocator<void>>::ConstPtr& message,
                                        const image_transport::SubscriberPlugin::Callback& user_cb)
{
  get_decoder()->push_data(message->data, message->header.stamp.toNSec());

  if (!get_decoder()->pull_data()) {
    return;
  }
  
  int width = get_decoder()->get_width();
  int height = get_decoder()->get_height();
  
  // Ensure dimensions are valid
  if (width <= 0 || height <= 0) {
    ROS_ERROR("Invalid image dimensions: %dx%d", width, height);
    return;
  }
  
  // Set fixed step for BGR8 (3 bytes per pixel, no padding)
  int step = width * 3;
  
  // Create new image with correct dimensions
  out_->data = get_decoder()->get_data();
  out_->width = width;
  out_->height = height;
  out_->step = step;
  out_->encoding = sensor_msgs::image_encodings::BGR8;
  
  // Verify data size matches dimensions
  size_t expected_size = step * height;
  if (out_->data.size() != expected_size) {
    ROS_ERROR("Image data size mismatch: got %zu bytes, expected %zu bytes", 
              out_->data.size(), expected_size);
    return;
  }
  
  out_->header = message->header;
  out_->header.stamp = ros::Time().fromNSec(get_decoder()->get_timestamp());
  user_cb(out_);
}
