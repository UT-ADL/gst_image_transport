#include "gst_image_transport/parent_subscriber.h"

namespace h264_image_transport
{

/**
 * Implementation-only h264 subscriber class.
 */
class H264Subscriber : public ParentSubscriber
{
public:
  /**
   * Get a string identifier for the transport provided by this subscriber.
   */
  std::string getTransportName() const override
  {
    return TRANSPORT_NAME;
  }

protected:
  /**
   * Gets the gst-decoder.
   */
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