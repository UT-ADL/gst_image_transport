# modern_image_transport

<sup>Inspired from [tilk/h264_image_transport](https://github.com/tilk/h264_image_transport).</sup>

H264 subscription plugin for the ROS image transport.  
This repo has been created to be used in pair with
the [nvidia_gmsl_driver_ros](https://github.com/UT-ADL/nvidia_gmsl_driver_ros) but can be used to decode any h264
packets published with the `sensor_msgs/CompressedImage` message.

## How to use

### Build

- Create a workspace
  ```bash
  mkdir -p catkin_ws/src
  cd catkin_ws
  ```
- Clone the repo
  ```bash
  git clone git@github.com:UT-ADL/gst_image_transport.git src/gst_image_transport
  ```
- Build and source the workspace
  ```bash
  catkin build
  source devel/setup.bash
  ```
- The H264 plugin for the ROS image transport is now available.

### Usage

- You can verify that the plugin is loaded correctly with :
  ```bash
  rosrun image_transport list_transports
  ```

  You should see :
  ```
    "image_transport/h264"
  - Provided by package: h264_image_transport
  - No publisher provided
  - Subscriber:
    This plugin decodes a h264 video stream.
  ```
- You can then decompress h264 messages like so :
  ```bash
  rosrun image_transport republish h264
  ```
  For details see the image transport republish [doc](http://wiki.ros.org/image_transport#Nodes).
