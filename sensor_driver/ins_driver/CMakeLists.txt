cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ins_driver)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    SYSTEM ${EIGEN3_INCLUDE_DIR}
)

add_library(ins_driver SHARED
    src/ins_driver.cpp
)

target_include_directories(ins_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(ins_driver
    gps
    network_driver
    cpp_utils
)
