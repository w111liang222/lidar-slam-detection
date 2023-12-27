
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cpp_sample)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(network)

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ./detection.proto)

add_executable(recv_detection_udp
               recv_detection_udp.cpp
               ${PROTO_SRCS}
               ${PROTO_HDRS}
)

target_link_libraries(recv_detection_udp
                      network_driver
                      ${PROTOBUF_LIBRARIES}
                      pthread
)

add_executable(recv_localization_udp
               recv_localization_udp.cpp
)

target_link_libraries(recv_localization_udp
                      network_driver
                      pthread
)