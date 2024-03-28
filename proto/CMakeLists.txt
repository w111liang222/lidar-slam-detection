cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(proto_detection)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Protobuf REQUIRED)

include_directories(
    ${PROTOBUF_INCLUDE_DIRS}
)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/detection.pb.cc")
  execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/generate_proto.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

add_library(proto_detection SHARED
    detection.pb.cc
)

target_include_directories(proto_detection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(proto_detection
    ${PROTOBUF_LIBRARIES}
)