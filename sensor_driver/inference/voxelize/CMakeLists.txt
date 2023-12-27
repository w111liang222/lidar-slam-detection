cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(CUDA)
if(CUDA_FOUND)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    project(voxelization LANGUAGES CXX CUDA)
    add_definitions(-DHAVE_CUDA_ENABLE)
    set(VOXELIZE_NVCC_FILES   "voxelization_kernel.cu")
    set(PREPROCESS_NVCC_FILES "preprocess_kernel.cu")
else()
    project(voxelization LANGUAGES CXX)
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
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
)

add_library(voxelization SHARED
    voxelization.cpp
    ${VOXELIZE_NVCC_FILES}
    ${PREPROCESS_NVCC_FILES}
)

target_include_directories(voxelization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(voxelization
    ${CUDA_LIBRARIES}
)
