cmake_minimum_required(VERSION 2.8.3)
project(h264_image_transport)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  cv_bridge
  pluginlib
  h264_image_transport_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES h264_image_transport
  CATKIN_DEPENDS message_runtime pluginlib
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(h264_image_transport
  src/h264_subscriber.cpp
  src/manifest.cpp
)

add_dependencies(h264_image_transport ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(h264_image_transport ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} x264 avcodec avutil swscale)

class_loader_hide_library_symbols(h264_image_transport)


## Mark executables and/or libraries for installation
install(TARGETS h264_image_transport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  h264_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)