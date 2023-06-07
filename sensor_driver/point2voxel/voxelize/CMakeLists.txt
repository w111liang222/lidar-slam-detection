cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(CUDA)
if(CUDA_FOUND)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    project(voxelization LANGUAGES CXX CUDA)
    add_definitions(-DHAS_CUDA_ENABLE)
    set(NVCC_FILES "voxelization_kernel.cu")
else()
    project(voxelization LANGUAGES CXX)
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")


include_directories (/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
    SYSTEM ${CUDA_INCLUDE_DIRS}
)

add_library(voxelization SHARED
    voxelization.cpp
    ${NVCC_FILES}
)

target_include_directories(voxelization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(voxelization
    ${CUDA_LIBRARIES}
)
