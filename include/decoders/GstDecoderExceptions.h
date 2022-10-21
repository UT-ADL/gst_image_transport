#pragma once

#include <stdexcept>

class GstDecoderInitException : public std::runtime_error
{
public:
  using runtime_error::runtime_error;
};

class GstDecoderRuntimeException : public std::runtime_error
{
public:
  using runtime_error::runtime_error;
};