cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(calib_driver)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

SET(NLOPT_DIR ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(NLOPT REQUIRED)

include_directories(
    SYSTEM ${EIGEN3_INCLUDE_DIR}
)

include_directories(
    lidar_ins/include
    lidar_imu/include
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}
    SYSTEM ${OPENCV_INCLUDE_DIRS}
)

add_library(calib_driver SHARED
    lidar_ins/src/sensor.cpp
    lidar_ins/src/aligner.cpp
    lidar_imu/src/calib_lidar_imu.cpp
)

target_include_directories(calib_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(calib_driver
    ${PCL_LIBRARIES}
    ${NLOPT_LIBRARIES}
    logging
    g2o_backend
)

