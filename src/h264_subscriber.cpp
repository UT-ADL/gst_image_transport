#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "h264_image_transport/h264_subscriber.h"

using namespace std;

namespace h264_image_transport {

H264Subscriber::H264Subscriber()
{
    av_init_packet(&avpkt_);
    avcodec_register_all();
    av_log_set_level(AV_LOG_FATAL);
    codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec_) {
        ROS_ERROR("cannot find H.264 codec_ in ffmpeg");
        throw ros::Exception("H264 codec_ not found, check that your ffmpeg is built with H264 support");
    }
    context_ = avcodec_alloc_context3(codec_);
    if(avcodec_open2(context_, codec_, nullptr) < 0) {
        ROS_ERROR("Cannot open codec_");
        throw ros::Exception("Cannot open codec_");
    }
    picture_ = av_frame_alloc();
}

H264Subscriber::~H264Subscriber()
{
    avcodec_close(context_);
    av_free(context_);
    av_frame_free(&picture_);
}

void H264Subscriber::subscribeImpl (ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size, const Callback &callback, const ros::VoidPtr &tracked_object, const image_transport::TransportHints &transport_hints)
{
    queue_size = 10;
    SimpleSubscriberPlugin::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
}

void H264Subscriber::internalCallback(const sensor_msgs::CompressedImage::ConstPtr& message, const Callback& user_cb)
{
    avpkt_.size = message->data.size();
    avpkt_.data = const_cast<uint8_t *>(message->data.data());

    if (avcodec_send_packet(context_, &avpkt_) != 0) {
        ROS_ERROR_STREAM_THROTTLE(5, "Failed to feed h264 frame to ffmpeg");
    }

    if (avcodec_receive_frame(context_, picture_) != 0) {
        // No frame
        return;
    }

    convert_ctx = sws_getCachedContext(convert_ctx, picture_->width, picture_->height, AV_PIX_FMT_YUV420P, picture_->width, picture_->height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR,
                                       nullptr, nullptr, nullptr);
    out_->data.resize(picture_->width * picture_->height * 3);
    int stride = 3 * picture_->width;
    dst_ = out_->data.data();
    sws_scale(convert_ctx, (const uint8_t* const*)picture_->data, picture_->linesize, 0, picture_->height, &dst_, &stride);

    out_->width = picture_->width;
    out_->height = picture_->height;
    out_->step = 3 * picture_->width;
    out_->encoding = sensor_msgs::image_encodings::BGR8;
    out_->header = message->header;
    user_cb(out_);
}
};

