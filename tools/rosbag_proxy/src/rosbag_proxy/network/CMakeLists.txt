cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(network_driver)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")


include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Boost REQUIRED COMPONENTS system filesystem)

set( BOOST_THREAD_LIBRARY "-lboost_thread")
set( Boost_SYSTEM_LIBRARY "-lboost_system")

include_directories(
    SYSTEM ${Boost_INCLUDE_DIRS}
)

add_library(network_driver SHARED
    UDPServer.cpp
)

target_include_directories(network_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(network_driver
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)

