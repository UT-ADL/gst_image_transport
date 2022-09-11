#ifndef H264_IMAGE_TRANSPORT_SUBSCRIBER_H
#define H264_IMAGE_TRANSPORT_SUBSCRIBER_H

#include <image_transport/simple_subscriber_plugin.h>
#include <image_transport/transport_hints.h>
#include <sensor_msgs/CompressedImage.h>

extern "C"
{
#include "libswscale/swscale.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"
}

namespace h264_image_transport {

class H264Subscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
    H264Subscriber();
    virtual ~H264Subscriber();

    virtual std::string getTransportName() const
    {
        return "h264";
    }

protected:
    virtual void internalCallback(const sensor_msgs::CompressedImage::ConstPtr& message, const Callback& user_cb);
    virtual void subscribeImpl (ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size, const Callback &callback, const ros::VoidPtr &tracked_object, const image_transport::TransportHints &transport_hints);

private:
    struct SwsContext *convert_ctx = nullptr;

    AVCodec *codec;
    AVCodecContext *c;
    AVFrame *picture;
    AVPacket avpkt;
};

};

#endif
