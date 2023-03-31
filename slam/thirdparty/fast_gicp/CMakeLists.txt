cmake_minimum_required(VERSION 3.10.0)
project(fast_gicp)

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DHAS_CUDA_ENABLE)
    set(BUILD_VGICP_CUDA "ON")
else()
    set(BUILD_VGICP_CUDA "OFF")
    set(CUDA_INCLUDE_DIRS "")
    set(CUDA_LIBRARIES "")
endif()

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
if( ${ARCHITECTURE} STREQUAL "x86_64")
    add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
    set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(${BUILD_VGICP_CUDA} STREQUAL "ON")
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

###########
## Build ##
###########

add_library(fast_gicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
)
target_link_libraries(fast_gicp
  ${PCL_LIBRARIES}
)
target_include_directories(fast_gicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

### CUDA ###
if(${BUILD_VGICP_CUDA} STREQUAL "ON")
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_72,code=sm_72")
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_87,code=sm_87")

  set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr ${CUDA_NVCC_FLAGS} -Xcompiler -fPIC -Xcudafe --diag_suppress=esa_on_defaulted_function_ignored")
  add_definitions(-DUSE_VGICP_CUDA)

  cuda_add_library(fast_vgicp_cuda SHARED
    src/fast_gicp/cuda/fast_vgicp_cuda.cu
    src/fast_gicp/cuda/brute_force_knn.cu
    src/fast_gicp/cuda/covariance_estimation.cu
    src/fast_gicp/cuda/covariance_estimation_rbf.cu
    src/fast_gicp/cuda/covariance_regularization.cu
    src/fast_gicp/cuda/gaussian_voxelmap.cu
    src/fast_gicp/cuda/find_voxel_correspondences.cu
    src/fast_gicp/cuda/compute_derivatives.cu
    src/fast_gicp/cuda/compute_mahalanobis.cu
    src/fast_gicp/cuda/ndt_cuda.cu
    src/fast_gicp/cuda/ndt_compute_derivatives.cu
  )
  target_include_directories(fast_vgicp_cuda PRIVATE
    include
    thirdparty/Eigen
    thirdparty/nvbio
  )
  cuda_add_cublas_to_target(fast_vgicp_cuda)

  # add vgicp_cuda to libfast_gicp
  target_sources(fast_gicp PRIVATE
    src/fast_gicp/gicp/fast_vgicp_cuda.cpp
    src/fast_gicp/ndt/ndt_cuda.cpp
  )
  target_link_libraries(fast_gicp
    fast_vgicp_cuda
  )
  add_dependencies(fast_gicp fast_vgicp_cuda)
endif()
