cmake_minimum_required(VERSION 3.10)

project(rosbag_writter LANGUAGES CXX)
 
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  pcl_ros
  nav_msgs
  sensor_msgs
  cv_bridge
  )

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ..)

add_library(rosbag_writter SHARED
    src/RosbagWritter.cpp
    )

target_link_libraries(rosbag_writter
    ${catkin_LIBRARIES}
    )
