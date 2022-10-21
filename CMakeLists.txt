cmake_minimum_required(VERSION 2.8.3)
project(gst_image_transport)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(catkin REQUIRED COMPONENTS image_transport cv_bridge pluginlib
        roscpp sensor_msgs)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        gst_image_transport
        CATKIN_DEPENDS
        message_runtime
        pluginlib
)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

include_directories(
        ${GLIB_INCLUDE_DIRS}
        ${GSTREAMER_INCLUDE_DIRS}
        ${GSTREAMER_APP_INCLUDE_DIRS}
)

link_directories(
        ${GLIB_LIBRARY_DIRS}
        ${GSTREAMER_LIBRARY_DIRS}
        ${GSTREAMER_APP_LIBRARY_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(gst_decoder src/decoders/GstDecoder.cpp)
add_library(gst_image_transport src/h264_subscriber.cpp src/h265_subscriber.cpp src/parent_subscriber.cpp)

add_dependencies(gst_image_transport ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(
        gst_decoder
        ${GSTREAMER_LIBRARIES})

target_link_libraries(
        gst_image_transport
        ${catkin_LIBRARIES}
        gst_decoder
        avcodec
        avutil
        swscale
        gstapp-1.0)

class_loader_hide_library_symbols(gst_image_transport)

install(
        TARGETS gst_image_transport
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(FILES image_transport_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
