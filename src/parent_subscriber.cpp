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
  out_->data = get_decoder()->get_data();
  out_->width = get_decoder()->get_width();
  out_->height = get_decoder()->get_height();
  out_->step = 3 * get_decoder()->get_width();
  out_->encoding = sensor_msgs::image_encodings::BGR8;
  out_->header = message->header;
  out_->header.stamp = ros::Time().fromNSec(get_decoder()->get_timestamp());
  user_cb(out_);
}
