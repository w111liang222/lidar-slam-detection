cmake_minimum_required(VERSION 3.6)
project(freespace_detection_wrapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(Protobuf REQUIRED)
include_directories(
  ${PYTHON_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ../proto
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../proto/detection.pb.cc")
  execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/generate_proto.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

pybind11_add_module(freespace_detection_wrapper freespace_detection_wrapper.h freespace_detection_wrapper.cpp 
                    freespace_detection.h freespace_detection.cc
                    coarse_segmentation/coarse_segmentation.h coarse_segmentation/coarse_segmentation.cc
                    common/common.h common/common.cc common/parameters.h common/types.h
                    preprocess/preprocess.h preprocess/preprocess.cc
                    remove_ground/patch_ground_plane_fiting.h remove_ground/patch_ground_plane_fiting.cc
                    ../proto/detection.pb.cc)
set_target_properties(freespace_detection_wrapper PROPERTIES PREFIX "")
set_target_properties(freespace_detection_wrapper PROPERTIES SUFFIX ".so")

target_link_libraries(freespace_detection_wrapper
  PRIVATE ${PYTHON_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${PCL_LIBRARIES}
)
