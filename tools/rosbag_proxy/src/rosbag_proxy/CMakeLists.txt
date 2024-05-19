cmake_minimum_required(VERSION 2.8.3)
project(rosbag_proxy)

add_subdirectory(async_web_server_cpp)

add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros cv_bridge image_transport sensor_msgs)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   include
)

add_subdirectory(network)

add_executable(rosbag_imu_proxy
  rosbag_imu_proxy.cpp
)
add_dependencies(rosbag_imu_proxy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_imu_proxy
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  network_driver
)

add_executable(rosbag_lidar_proxy
  rosbag_lidar_proxy.cpp
)
add_dependencies(rosbag_lidar_proxy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_lidar_proxy
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  network_driver
)

add_executable(rosbag_camera_proxy
  rosbag_camera_proxy.cpp
  src/web_video_server.cpp
  src/image_streamer.cpp
  src/multipart_stream.cpp
  src/ros_compressed_streamer.cpp
  src/jpeg_streamers.cpp
)

add_dependencies(rosbag_camera_proxy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_camera_proxy
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  async_web_server_cpp
)