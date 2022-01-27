cmake_minimum_required(VERSION 2.8.3)
project(h264_image_transport_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )

add_message_files(
        FILES
        H264Packet.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
