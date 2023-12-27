cmake_minimum_required(VERSION 3.10)

project(pkl_to_rosbag LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(rosbag_writter)
 
set(CMAKE_CXX_STANDARD 14)

include_directories(/usr/include)
link_directories(/usr/local/lib)

find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIR})

find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

include_directories(
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    rosbag_writter/include
    pybind11/include
  )

add_executable (pkl_to_rosbag src/main.cpp src/GetPklData.cpp)
target_link_libraries (pkl_to_rosbag  rosbag_writter ${PYTHON_LIBRARY})