#include "gst_image_transport/parent_subscriber.h"

namespace h264_image_transport
{

class H264Subscriber : public ParentSubscriber
{
public:
  std::string getTransportName() const override
  {
    return TRANSPORT_NAME;
  }

protected:
  GstDecoder* get_decoder() override
  {
    return decoder_.get();
  }

private:
  static inline const std::string TRANSPORT_NAME = "h264";
  std::unique_ptr<GstDecoder> decoder_ = std::make_unique<GstDecoder>(TRANSPORT_NAME);
};

};  // namespace h264_image_transport

PLUGINLIB_EXPORT_CLASS(h264_image_transport::H264Subscriber, image_transport::SubscriberPlugin);