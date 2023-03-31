cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(fast_lio)

set(CMAKE_C_FLAGS "-Wall -fexceptions")
set(CMAKE_CXX_FLAGS "-std=c++17 -fexceptions")
set(CMAKE_BUILD_TYPE "Release")

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Architecture: ${ARCHITECTURE}")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

include(ProcessorCount)
ProcessorCount(N)
message("Processer number:  ${N}")
if( ${ARCHITECTURE} STREQUAL "x86_64")
    set(N 8)
endif()
message("FastLio Use Processer number:  ${N}")
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=${N})

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    SYSTEM ${EIGEN3_INCLUDE_DIR}
    SYSTEM ${Boost_INCLUDE_DIRS}
    SYSTEM ${PCL_INCLUDE_DIRS}
)

add_library(fast_lio SHARED
    src/fastlio.cpp
    src/laserMapping.cpp
    include/ikd-Tree/ikd_Tree.cpp
    src/preprocess.cpp
)

target_include_directories(fast_lio
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


target_link_libraries(fast_lio
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    map_common
)
