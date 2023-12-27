cmake_minimum_required(VERSION 3.6)
project(hdl_graph_slam)

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DUSE_VGICP_CUDA)
else()
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

set(CMAKE_C_FLAGS "-Wall -Wno-narrowing")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wno-narrowing -pthread")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" ${CMAKE_MODULE_PATH})

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


###########
## Build ##
###########
include_directories(include apps)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_library(g2o_backend SHARED
  apps/prefiltering_nodelet.cpp
  apps/scan_matching_odometry_nodelet.cpp
  apps/floor_detection_nodelet.cpp
  apps/hdl_graph_slam_nodelet.cpp
  src/hdl_graph_slam/registrations.cpp
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/keyframe.cpp
  src/hdl_graph_slam/registrations.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
)

target_include_directories(g2o_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(g2o_backend
  fast_gicp
  map_common
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
  ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)
