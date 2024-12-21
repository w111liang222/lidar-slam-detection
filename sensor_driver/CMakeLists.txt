cmake_minimum_required(VERSION 3.5)
project(sensor_driver)

set(CMAKE_C_FLAGS "-Wall -Wno-narrowing")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wno-narrowing")
set(CMAKE_BUILD_TYPE "Release")

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Architecture: ${ARCHITECTURE}")

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DHAVE_CUDA_ENABLE)
else()
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex)
find_package(OpenCV2 REQUIRED)

include_directories(
    SYSTEM ${EIGEN3_INCLUDE_DIR}
    SYSTEM ${Boost_INCLUDE_DIRS}
    SYSTEM ${CUDA_INCLUDE_DIRS}
    SYSTEM ${OPENCV_INCLUDE_DIRS}
)

add_subdirectory(common_lib)
add_subdirectory(inference)
add_subdirectory(lidar_driver)
add_subdirectory(ins_driver)
add_subdirectory(radar_driver)
add_subdirectory(calibration)


pybind11_add_module(lidar_driver_ext lidar_driver/lidar_driver_wrapper.h lidar_driver/lidar_driver_wrapper.cpp)
set_target_properties(lidar_driver_ext PROPERTIES PREFIX "")
set_target_properties(lidar_driver_ext PROPERTIES SUFFIX ".so")
target_link_libraries(lidar_driver_ext
    PRIVATE ${PYTHON_LIBARIES}
    lidar_driver
)

pybind11_add_module(ins_driver_ext ins_driver/ins_driver_wrapper.h ins_driver/ins_driver_wrapper.cpp)
set_target_properties(ins_driver_ext PROPERTIES PREFIX "")
set_target_properties(ins_driver_ext PROPERTIES SUFFIX ".so")
target_link_libraries(ins_driver_ext
    PRIVATE ${PYTHON_LIBARIES}
    ins_driver
)

pybind11_add_module(radar_driver_ext radar_driver/radar_driver_wrapper.h radar_driver/radar_driver_wrapper.cpp)
set_target_properties(radar_driver_ext PROPERTIES PREFIX "")
set_target_properties(radar_driver_ext PROPERTIES SUFFIX ".so")
target_link_libraries(radar_driver_ext
    PRIVATE ${PYTHON_LIBARIES}
    radar_driver
)

pybind11_add_module(cpp_utils_ext common_lib/cpp_utils/cpp_utils.h common_lib/cpp_utils/all.cpp)
set_target_properties(cpp_utils_ext PROPERTIES PREFIX "")
set_target_properties(cpp_utils_ext PROPERTIES SUFFIX ".so")
target_link_libraries(cpp_utils_ext
    PRIVATE ${PYTHON_LIBARIES}
    ${CUDA_LIBRARIES}
    cpp_utils
)

pybind11_add_module(inference_ext inference/inference.h inference/all.cpp)
set_target_properties(inference_ext PROPERTIES PREFIX "")
set_target_properties(inference_ext PROPERTIES SUFFIX ".so")
target_link_libraries(inference_ext
    PRIVATE ${PYTHON_LIBARIES}
    inference
)

pybind11_add_module(iou3d_nms_ext inference/iou3d_nms/iou3d_nms_api.h inference/iou3d_nms/iou3d_nms_api.cpp)
set_target_properties(iou3d_nms_ext PROPERTIES PREFIX "")
set_target_properties(iou3d_nms_ext PROPERTIES SUFFIX ".so")
target_link_libraries(iou3d_nms_ext
    PRIVATE ${PYTHON_LIBARIES}
    iou3d_nms
)

pybind11_add_module(calib_driver_ext calibration/calib_wrapper.h calibration/calib_wrapper.cpp)
set_target_properties(calib_driver_ext PROPERTIES PREFIX "")
set_target_properties(calib_driver_ext PROPERTIES SUFFIX ".so")
target_link_libraries(calib_driver_ext
    PRIVATE ${PYTHON_LIBARIES}
    calib_driver
)