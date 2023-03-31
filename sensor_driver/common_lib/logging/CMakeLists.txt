cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(logging)

add_subdirectory(message)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
    message
)

add_library(logging SHARED
    Logger.cpp
    InterProcess.cpp
)

target_include_directories(logging
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/message)

target_link_libraries(logging
    zcm
)
