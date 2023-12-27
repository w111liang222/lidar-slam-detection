cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(map_common)

add_subdirectory(DBoW2)

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
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OPENCV_INCLUDE_DIRS}
)

add_library(map_common SHARED
  slam_utils.cpp
  pcd_writer.cpp
  keyframe.cpp
  Scancontext/Scancontext.cpp
  # ORB SLAM
  ORBSLAM/src/Converter.cc
  ORBSLAM/src/Frame.cc
  ORBSLAM/src/Initializer.cc
  ORBSLAM/src/KeyFrame.cc
  ORBSLAM/src/KeyFrameDatabase.cc
  ORBSLAM/src/Map.cc
  ORBSLAM/src/MapPoint.cc
  ORBSLAM/src/ORBextractor.cc
  ORBSLAM/src/ORBmatcher.cc
)

target_include_directories(map_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(map_common
  ${PCL_LIBRARIES}
  ${OPENCV_LIB}
  network_driver
  cpp_utils
  DBoW2
)
