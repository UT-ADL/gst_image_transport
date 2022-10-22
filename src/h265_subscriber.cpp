#include "gst_image_transport/parent_subscriber.h"

namespace h265_image_transport
{

/**
 * Implementation-only h265 subscriber class.
 */
class H265Subscriber : public ParentSubscriber
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
  static inline const std::string TRANSPORT_NAME = "h265";
  std::unique_ptr<GstDecoder> decoder_ = std::make_unique<GstDecoder>(TRANSPORT_NAME);
};

};  // namespace h265_image_transport

PLUGINLIB_EXPORT_CLASS(h265_image_transport::H265Subscriber, image_transport::SubscriberPlugin);