cmake_minimum_required(VERSION 3.10)

project(rosbag_to_pkl LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(rosbag_reader)
 
set(CMAKE_CXX_STANDARD 14)

include_directories(/usr/include)
link_directories(/usr/local/lib)

find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIR})

find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include_directories(
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    rosbag_reader/include
    pybind11/include
  )

add_executable (rosbag_to_pkl src/main.cpp src/Utils.cpp)
target_link_libraries (rosbag_to_pkl  rosbag_reader ${PYTHON_LIBRARY})