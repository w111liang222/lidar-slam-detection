cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(gicpm)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")


include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    SYSTEM ${EIGEN3_INCLUDE_DIR}
    SYSTEM ${PCL_INCLUDE_DIRS}
)

add_library(gicpm SHARED
    src/gicpm.cpp
)

target_include_directories(gicpm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


target_link_libraries(gicpm
    ${PCL_LIBRARIES}
    map_common
)
