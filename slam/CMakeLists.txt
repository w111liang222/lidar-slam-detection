cmake_minimum_required(VERSION 3.6)
project(slam_wrapper)

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DHAVE_CUDA_ENABLE)
else()
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

add_subdirectory(thirdparty/fast_gicp)
add_subdirectory(thirdparty/fast_max-clique_finder)

set(CMAKE_C_FLAGS "-Wall -Wno-narrowing")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wno-narrowing -pthread")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV2 REQUIRED)

include_directories(
    common
    ${CMAKE_CURRENT_SOURCE_DIR}/backend
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
    SYSTEM ${EIGEN3_INCLUDE_DIR}
    SYSTEM ${Boost_INCLUDE_DIRS}
    SYSTEM ${PCL_INCLUDE_DIRS}
    SYSTEM ${OPENCV_INCLUDE_DIRS}
)

add_subdirectory(common)
add_subdirectory(backend/hdl_graph_slam)
add_subdirectory(mapping/rtkm)
add_subdirectory(mapping/fastlio)
add_subdirectory(localization)

add_library(slam SHARED
  src/slam.cpp
)

target_link_libraries(slam
  rtkm
  fast_lio
  g2o_backend
  localization
)

pybind11_add_module(slam_wrapper
  src/slam_wrapper.h
  src/slam_wrapper.cpp
  src/graph_utils.h
  src/graph_utils.cpp
  src/py_utils.h
  src/py_utils.cpp
)
set_target_properties(slam_wrapper PROPERTIES PREFIX "")
set_target_properties(slam_wrapper PROPERTIES SUFFIX ".so")

target_link_libraries(slam_wrapper
  PRIVATE ${PYTHON_LIBRARIES}
  slam
)
