cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(CUDA)
if(CUDA_FOUND)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    project(map_render LANGUAGES CXX CUDA)
    add_definitions(-DHAVE_CUDA_ENABLE)
    set(CUDA_SOURCE_FILES "cuda/image_render.cu")
else()
    project(map_render LANGUAGES CXX)
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_72,code=sm_72")
string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_80,code=sm_80")
string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_86,code=sm_86")
string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_87,code=sm_87")
if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 11.8)
    string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_89,code=sm_89")
endif()

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OPENCV_INCLUDE_DIRS}
  SYSTEM ${CUDA_INCLUDE_DIRS}
)

add_library(map_render SHARED
  map_render.cpp
  camera_model.cpp
  colmap_utils.cpp
  ${CUDA_SOURCE_FILES}
)

target_include_directories(map_render
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(map_render
  ${PCL_LIBRARIES}
  ${OPENCV_LIB}
  ${CUDA_LIBRARIES}
  map_common
)
