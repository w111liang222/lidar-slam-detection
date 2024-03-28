cmake_minimum_required(VERSION 3.6)
project(perception)

add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/pybind11)
include_directories(
    ${CMAKE_SOURCE_DIR}/third_party/readerwriterqueue
)

message(STATUS "BOARD_NAME: ${BOARD_NAME}")
EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
if(${ARCHITECTURE} STREQUAL "x86_64")
    add_definitions(-DPLATFORM_X86_64)
else()
    add_definitions(-DPLATFORM_AARCH64)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/slam/cmake)

add_subdirectory(proto)
add_subdirectory(sensor_driver)
add_subdirectory(slam)
