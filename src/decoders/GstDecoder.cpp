#include "decoders/GstDecoder.h"

GstDecoder::GstDecoder(std::string_view const& transportName) : transport_name_(transportName)
{
  if (G_UNLIKELY(!TRANSPORT_TO_PIPELINE_DESC.count(transport_name_))) {
    throw GstDecoderInitException("Transport " + transport_name_ + " is not supported.");
  }

  gst_init(nullptr, nullptr);

  std::string launchstr = TRANSPORT_TO_PIPELINE_DESC.at(transport_name_) + SUFFIX;
  bin_ = gst_parse_launch(launchstr.c_str(), &err_);

  if (G_UNLIKELY(bin_ == nullptr)) {
    throw GstDecoderInitException("Couldn't create bin from launch string : '" + launchstr + "'.");
  }

  if (G_UNLIKELY(err_ != nullptr)) {
    throw GstDecoderInitException("Error while parsing launch string : " + std::string(err_->message));
  }

  configure_app_elements();
  timestamp_caps_ = gst_caps_from_string("application/timestamp");
  gst_element_set_state(bin_, GST_STATE_PLAYING);
}

GstDecoder::~GstDecoder()
{
  gst_element_set_state(bin_, GST_STATE_NULL);
  gst_element_get_state(bin_, nullptr, nullptr, GST_CLOCK_TIME_NONE);
  gst_object_unref(bin_);
  gst_caps_unref(timestamp_caps_);
}

void GstDecoder::configure_app_elements()
{
  appsrc_element_ = gst_bin_get_by_name(GST_BIN(bin_), "appsrc");

  if (G_UNLIKELY(appsrc_element_ == nullptr)) {
    throw GstDecoderInitException("Couldn't retrieve appsrc.");
  }

  appsink_element_ = gst_bin_get_by_name(GST_BIN(bin_), "appsink");

  if (G_UNLIKELY(appsink_element_ == nullptr)) {
    throw GstDecoderInitException("Couldn't retrieve appsink.");
  }
}

void GstDecoder::push_data(const std::vector<uint8_t>& data, uint64_t timestamp)
{
  in_buffer_ = gst_buffer_new_allocate(nullptr, data.size(), nullptr);

  if (G_UNLIKELY(!in_buffer_)) {
    throw GstDecoderRuntimeException("Failed to allocate buffer for " + std::to_string(data.size()) + " bytes.");
  }

  gst_buffer_fill(in_buffer_, 0, data.data(), data.size());

  gst_buffer_add_reference_timestamp_meta(in_buffer_, timestamp_caps_, timestamp, GST_CLOCK_TIME_NONE);

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_element_), in_buffer_);

  if (G_UNLIKELY(ret != GST_FLOW_OK)) {
    ROS_ERROR_STREAM("Error while pushing buffer: " << (int)ret);
  }
}

bool GstDecoder::pull_data()
{
  out_sample_ = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_element_), 0);

  if (G_UNLIKELY(out_sample_ == nullptr)) {
    return false;
  }

  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(bin_), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

  if (G_UNLIKELY(width_ == 0 || height_ == 0)) {
    auto const* caps = gst_sample_get_caps(out_sample_);
    auto const* structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);
  }

  out_buffer_ = gst_sample_get_buffer(out_sample_);
  if (GstMapInfo map; G_LIKELY(gst_buffer_map(out_buffer_, &map, GST_MAP_READ) != 0)) {
    out_data_.assign(map.data, map.data + map.size);
    gst_buffer_unmap(out_buffer_, &map);
  }

  out_timestamp_ = gst_buffer_get_reference_timestamp_meta(out_buffer_, timestamp_caps_)->timestamp;

  gst_sample_unref(out_sample_);
  return true;
}

std::vector<uint8_t> GstDecoder::get_data()
{
  return std::move(out_data_);
}

uint64_t GstDecoder::get_timestamp() const
{
  return out_timestamp_;
}

int GstDecoder::get_width() const
{
  return width_;
}

int GstDecoder::get_height() const
{
  return height_;
}