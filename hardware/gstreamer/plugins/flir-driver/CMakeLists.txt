cmake_minimum_required(VERSION "3.6")
project(flir_driver)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
set(CMAKE_BUILD_TYPE "Release")

find_package(Spinnaker)

add_library(flir_driver
  spinnaker_camera_wrapper.cpp
  video_capture_spinnaker.cpp
)
target_link_libraries(flir_driver
  ${OPENCV_LIB}
  ${SPINNAKER_LIB}
)
target_include_directories(flir_driver PRIVATE ${SPINNAKER_INCLUDE_DIRS})
target_include_directories(flir_driver INTERFACE . )