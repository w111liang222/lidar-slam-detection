cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(localization)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/hdl_localization/include
    SYSTEM ${EIGEN3_INCLUDE_DIR}
    SYSTEM ${Boost_INCLUDE_DIRS}
    SYSTEM ${PCL_INCLUDE_DIRS}
)

add_subdirectory(map_render)

add_library(localization SHARED
    src/localization.cpp
    src/global_localization.cpp
    src/map_loader.cpp
    hdl_localization/hdl_localization.cpp
    hdl_localization/src/pose_estimator.cpp
    hdl_localization/apps/hdl_localization_nodelet.cpp
)

target_include_directories(localization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include)

target_link_libraries(localization
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    fast_max-clique_finder
    g2o_backend
    map_render
    map_common
)
