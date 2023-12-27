cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(visual_odometry)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rgb_map
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OPENCV_INCLUDE_DIRS}
)

add_library(visual_odometry SHARED
  src/visual_odometry.cpp
  rgb_map/image_frame.cpp
  rgb_map/pointcloud_rgbd.cpp
  rgb_map/rgbmap_tracker.cpp
)

target_include_directories(visual_odometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rgb_map>)

target_link_libraries(visual_odometry
  ${PCL_LIBRARIES}
  ${OPENCV_LIB}
  map_common
)
