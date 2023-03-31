cmake_minimum_required(VERSION 3.6)
project(perception)

add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/pybind11)

include_directories(
    ${CMAKE_SOURCE_DIR}/third_party/readerwriterqueue
)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
if(${ARCHITECTURE} STREQUAL "x86_64")
    add_definitions(-DPLATFORM_X86_64)
else()
    add_definitions(-DPLATFORM_AARCH64)
endif()

message(STATUS "BOARD_NAME: ${BOARD_NAME}")
if(${BOARD_NAME} STREQUAL "Xavier-NX")
    add_definitions(-DXavierNX)
elseif(${BOARD_NAME} STREQUAL "Xavier-AGX")
    add_definitions(-DXavierAGX)
elseif(${BOARD_NAME} STREQUAL "AGX-Orin-32GB")
    add_definitions(-DAGXOrin32GB)
elseif(${BOARD_NAME} STREQUAL "IPC")
    add_definitions(-DINTELIPC)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/slam/cmake)

add_subdirectory(sensor_driver)
add_subdirectory(freespace_detection)
add_subdirectory(slam)
