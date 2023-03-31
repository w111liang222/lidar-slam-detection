cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(cpp_utils)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV2 REQUIRED)

include_directories(
    SYSTEM ${EIGEN3_INCLUDE_DIR}
    SYSTEM ${Boost_INCLUDE_DIRS}
    SYSTEM ${PCL_INCLUDE_DIRS}
    SYSTEM ${OPENCV_INCLUDE_DIRS}
)

add_library(cpp_utils SHARED
    SystemUtils.cpp
    Utils.cpp
    UTMProjector.cpp
    KalmanFilter.cpp
)

target_include_directories(cpp_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(cpp_utils
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OPENCV_LIB}
    logging
    network_driver
)
