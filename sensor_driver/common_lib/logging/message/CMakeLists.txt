cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(message)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
)

add_library(message SHARED
    std_msgs/std_msgs_Byte.c
    std_msgs/std_msgs_Float.c
    std_msgs/std_msgs_Header.c
    std_msgs/std_msgs_Int32.c
    std_msgs/std_msgs_Int64.c
    std_msgs/std_msgs_String.c

    geometry_msgs/geometry_msgs_Point.c
    geometry_msgs/geometry_msgs_Point32.c
    geometry_msgs/geometry_msgs_Pose.c
    geometry_msgs/geometry_msgs_PoseStamped.c
    geometry_msgs/geometry_msgs_PoseWithCovariance.c
    geometry_msgs/geometry_msgs_Quaternion.c
    geometry_msgs/geometry_msgs_Twist.c
    geometry_msgs/geometry_msgs_TwistWithCovariance.c
    geometry_msgs/geometry_msgs_Vector3.c

    nav_msgs/nav_msgs_Odometry.c
    nav_msgs/nav_msgs_Path.c

    sensor_msgs/sensor_msgs_CompressedImage.c
    sensor_msgs/sensor_msgs_Image.c
    sensor_msgs/sensor_msgs_Imu.c
    sensor_msgs/sensor_msgs_NavSatFix.c
    sensor_msgs/sensor_msgs_NavSatStatus.c
    sensor_msgs/sensor_msgs_PointCloud.c
    sensor_msgs/sensor_msgs_PointField.c
)

target_include_directories(message
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(message
    zcm
)
