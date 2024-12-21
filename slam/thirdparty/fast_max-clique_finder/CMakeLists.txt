cmake_minimum_required(VERSION 3.6)
project(fast_max-clique_finder)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_BUILD_TYPE "RELEASE")

add_library(fast_max-clique_finder SHARED
  src/findClique.h
	src/graphIO.h
	src/findClique.cpp
	src/findCliqueHeu.cpp
	src/utils.cpp
	src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

target_include_directories(fast_max-clique_finder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/src)
