#pragma once

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros/ros.h"

class GstDecoder
{
public:
  GstDecoder(std::string_view const& transportName);
  virtual ~GstDecoder();
  void push_data(const std::vector<uint8_t>& data);
  bool pull_data();
  std::vector<uint8_t> get_data();

private:
  const static inline std::unordered_map<std::string, std::string> TRANSPORT_TO_PIPELINE_DESC{
    { "h264", "h264parse ! avdec_h264" },
    { "h265", "h265parse ! avdec_h265" },
    { "vp9", "vp9parse ! avdec_vp9" },
  };
  const static inline std::string PREFIX = "appsrc name=appsrc ! ";
  const static inline std::string SUFFIX = " ! videoconvert ! appsink name=appsink caps=video/x-raw,format=BGR";

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
  [[nodiscard]] int getWidth() const;
  [[nodiscard]] int getHeight() const;
};
