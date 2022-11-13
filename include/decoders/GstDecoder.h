#pragma once

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <ros/ros.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "GstDecoderExceptions.h"

class GstDecoder
{
public:
  /**
   * Constructor.
   * Sets up the pipelind accotding to the passed transportName.
   */
  explicit GstDecoder(std::string_view const& transportName);

  /**
   * Destructor.
   * Stops and unreferences the pipeline.
   */
  virtual ~GstDecoder();

  /**
   * Pushes data into the the pipeline.
   * @param data Data to push into the pipeline.
   * @param timestamp Timestamp in ns to push alongside the data.
   */
  void push_data(const std::vector<uint8_t>& data, uint64_t timestamp);

  /**
   * Attempts to pull data from the pipeline.
   * Returns true if successful, false otherwise.
   * The pulled data can be retrieved with get_data().
   */
  bool pull_data();

  /**
   * Returns a vector containing the data pulled from the pipeline by pull_data().
   */
  [[nodiscard]] std::vector<uint8_t> get_data();

  /**
   * Returns the timestamp attached to the data pulled from the pipeline by pull_data().
   */
  [[nodiscard]] uint64_t get_timestamp() const;

private:
  /**
   * Map containing the supported transports and their respective pipeline descriptions.
   * New transports should be added to this map.
   */
  const static inline std::unordered_map<std::string, std::string> TRANSPORT_TO_PIPELINE_DESC{ // clang-format off
    { "h264", "appsrc name=appsrc caps=video/x-h264,stream-format=byte-stream,alignment=au,interlace-mode=progressive"
      " ! queue ! h264parse ! queue ! avdec_h264 max-threads=1 ! queue" },
    { "h265", "appsrc name=appsrc caps=video/x-h265,stream-format=byte-stream,alignment=au ! queue ! h265parse ! queue "
      "! libde265dec ! queue" }
  };  // clang-format on
  const static inline std::string SUFFIX = " ! videoconvert ! appsink name=appsink caps=video/x-raw,format=BGR";

  /**
   * Configures the appsrc and appsink elements.
   */
  void configure_app_elements();

  GstElement* bin_;
  GstSample* out_sample_;
  GstBuffer* out_buffer_;
  GstBuffer* in_buffer_;
  GError* err_ = nullptr;
  GstElement* appsrc_element_;
  GstElement* appsink_element_;
  GstCaps* timestamp_caps_ = nullptr;

  int width_ = 0;
  int height_ = 0;
  std::string transport_name_;
  std::vector<uint8_t> out_data_;
  uint64_t out_timestamp_;

public:
  [[nodiscard]] int get_width() const;
  [[nodiscard]] int get_height() const;
};
