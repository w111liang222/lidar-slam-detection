cmake_minimum_required(VERSION "3.6")

project(gst-plugins-tsari)

set(CMAKE_SHARED_MODULE_PREFIX "lib")
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
set(CMAKE_BUILD_TYPE "Release")

# add local cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

find_package(GStreamer REQUIRED COMPONENTS base)
find_package(GStreamerPluginsBase COMPONENTS video)
find_package(GLIB2 REQUIRED)
find_package(GObject REQUIRED)
find_package(OpenCV2 REQUIRED)

# Setup common environment
include_directories(
    ${GSTREAMER_INCLUDE_DIR}
    ${GLIB2_INCLUDE_DIR}
    SYSTEM ${OPENCV_INCLUDE_DIRS}
)

# values used when registering plugins
add_definitions(-DGST_PACKAGE_VERSION="3.5")
add_definitions(-DGST_PACKAGE_LICENSE="LGPL")
add_definitions(-DGST_PACKAGE_ORIGIN="https://github.com/w111liang222/lidar-slam-detection")
add_definitions(-DGST_PACKAGE_NAME="${CMAKE_PROJECT_NAME}")
add_definitions(-DPACKAGE="${CMAKE_PROJECT_NAME} package")

add_subdirectory(flir-driver)
add_subdirectory(gst-flir-src)
add_subdirectory(gst-opencv-remap)
