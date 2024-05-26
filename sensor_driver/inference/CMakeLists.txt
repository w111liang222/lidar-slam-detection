cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_subdirectory(iou3d_nms)
add_subdirectory(voxelize)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)

find_package(CUDA)
if(CUDA_FOUND)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    project(inference LANGUAGES CXX CUDA)
    add_definitions(-DHAVE_CUDA_ENABLE)
    set(SRC_NV_FILES "tensorRT/tensorrt.cpp")
    set(NV_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libspconv/lib/${ARCHITECTURE}/libspconv.so libnvinfer.so)
else()
    project(inference LANGUAGES CXX)
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wno-deprecated-declarations")
set(CMAKE_BUILD_TYPE "Release")
string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_80,code=sm_80")
string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_86,code=sm_86")
string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_87,code=sm_87")
if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 11.8)
    string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_89,code=sm_89")
endif()

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
    SYSTEM ${CUDA_INCLUDE_DIRS}
    libspconv/include
)

add_library(inference SHARED
    tensorRT/lidar_inference.cpp
    ${SRC_NV_FILES}
)

target_include_directories(inference
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/tensorRT
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/libspconv/include)

target_link_libraries(inference
    ${CUDA_LIBRARIES}
    ${NV_LIBRARIES}
    voxelization
    cpp_utils
)
