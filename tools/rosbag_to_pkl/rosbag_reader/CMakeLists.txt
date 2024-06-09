cmake_minimum_required(VERSION 3.10)

project(rosbag_reader LANGUAGES CXX)
 
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(msg)

set(lsd_messages
    monitor_msgs_generate_messages
    velodyne_msgs_generate_messages
    livox_ros_driver_generate_messages
    )

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  pcl_ros
  nav_msgs
  nmea_msgs
  sensor_msgs
  cv_bridge
  )

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${CMAKE_BINARY_DIR}/devel/include
                    ..)

add_library(rosbag_reader SHARED
    src/cloud_convert.cpp
    src/nmea_converter_core.cpp
    src/RosbagReader.cpp
    )

target_link_libraries(rosbag_reader
    ${catkin_LIBRARIES}
    )

add_dependencies(rosbag_reader ${lsd_messages})