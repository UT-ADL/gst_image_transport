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
   */
  void push_data(const std::vector<uint8_t>& data);

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

private:
  /**
   * Map containing the supported transports and their respective pipeline descriptions.
   * New transports should be added to this map.
   */
  const static inline std::unordered_map<std::string, std::string> TRANSPORT_TO_PIPELINE_DESC{
    { "h264", "h264parse ! avdec_h264" },
    { "h265", "h265parse ! avdec_h265" }
  };
  const static inline std::string PREFIX = "appsrc name=appsrc ! ";
  const static inline std::string SUFFIX = " ! videoconvert ! appsink name=appsink caps=video/x-raw,format=BGR";

  /**
   * Configures the appsrc and appsink elements.
   */
  void configure_app_elements();

  GstElement* bin_;
  GstSample* OutSample_;
  GstBuffer* OutBuffer_;
  GstBuffer* InBuffer_;
  GError* err_ = nullptr;
  GstElement* appsrc_element_;
  GstElement* appsink_element_;

  int width_ = 0;
  int height_ = 0;
  std::string transport_name_;
  std::vector<uint8_t> OutData_;

public:
  [[nodiscard]] int get_width() const;
  [[nodiscard]] int get_height() const;
};
