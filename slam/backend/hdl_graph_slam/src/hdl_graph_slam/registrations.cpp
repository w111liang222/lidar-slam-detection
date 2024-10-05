// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/registrations.hpp>

#include <iostream>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

//#include <pclomp/ndt_omp.h>
//#include <pclomp/gicp_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

#include "Logger.h"

namespace hdl_graph_slam {

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method() {
  return select_registration_method("FAST_VGICP_CUDA");
}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method(std::string registration_method, int64_t max_process_time) {
  using PointT = pcl::PointXYZI;

  // select a registration method (ICP, GICP, NDT)
  if(registration_method.compare("FAST_GICP") == 0) {
    LOG_INFO("registration: FAST_GICP");
    fast_gicp::FastGICP<PointT, PointT>::Ptr gicp(new fast_gicp::FastGICP<PointT, PointT>());
    gicp->setNumThreads(4);
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setMaxCorrespondenceDistance(2.0);
    gicp->setCorrespondenceRandomness(20);
    return gicp;
  }
#ifdef USE_VGICP_CUDA
  else if(registration_method.compare("FAST_VGICP_CUDA") == 0) {
    LOG_INFO("registration: FAST_VGICP_CUDA");
    fast_gicp::FastVGICPCuda<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICPCuda<PointT, PointT>());
    vgicp->setResolution(1.0);
    vgicp->setTransformationEpsilon(0.01);
    vgicp->setMaximumIterations(64);
    vgicp->setCorrespondenceRandomness(20);
    // vgicp->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
    // vgicp->setKernelWidth(0.5);
    return vgicp;
  }
#endif
  else if(registration_method.compare("FAST_VGICP") == 0) {
    LOG_INFO("registration: FAST_VGICP");
    fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
    vgicp->setNumThreads(4);
    vgicp->setResolution(1.0);
    vgicp->setTransformationEpsilon(0.1);
    vgicp->setRotationEpsilon(0.1);
    vgicp->setMaximumIterations(64);
    vgicp->setCorrespondenceRandomness(20);
    vgicp->setMaxProcessTime(max_process_time);
    return vgicp;
  } else if(registration_method.compare("ICP") == 0) {
    LOG_INFO("registration: ICP");
    pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon(0.01);
    icp->setMaximumIterations(64);
    icp->setMaxCorrespondenceDistance(2.5);
    icp->setUseReciprocalCorrespondences(false);
    return icp;
  } else if(registration_method.find("GICP") != std::string::npos) {
    if(registration_method.find("OMP") == std::string::npos) {
      LOG_INFO("registration: GICP");
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(0.01);
      gicp->setMaximumIterations(64);
      gicp->setUseReciprocalCorrespondences(false);
      gicp->setMaxCorrespondenceDistance(5.0);
      gicp->setCorrespondenceRandomness(20);
      gicp->setMaximumOptimizerIterations(20);
      return gicp;
    } else {
      return nullptr;
      /*
      std::cout << "registration: GICP_OMP" << std::endl;
      pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(0.01);
      gicp->setMaximumIterations(64);
      gicp->setUseReciprocalCorrespondences(false);
      gicp->setMaxCorrespondenceDistance(2.5);
      gicp->setCorrespondenceRandomness(20);
      gicp->setMaximumOptimizerIterations(20);
      return gicp;
      */
    }
  } else {
    if(registration_method.find("NDT") == std::string::npos) {
      LOG_WARN("warning: unknown registration type({})", registration_method);
      LOG_WARN("       : use NDT");
    }

    double ndt_resolution = 1.0;
    if (registration_method.compare("NDT_CUDA") == 0) {
#ifdef USE_VGICP_CUDA
      LOG_INFO("registration: NDT_CUDA {}", ndt_resolution);
      fast_gicp::NDTCuda<PointT, PointT>::Ptr ndt(new fast_gicp::NDTCuda<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setRotationEpsilon(0.1);
      ndt->setMaximumIterations(64);
      ndt->setResolution(ndt_resolution);
      ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7, 0.0);
      ndt->setMaxProcessTime(max_process_time);
      return ndt;
#else
      return nullptr;
#endif
    } else if(registration_method.find("OMP") == std::string::npos) {
      LOG_INFO("registration: NDT {}", ndt_resolution);
      pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setMaximumIterations(64);
      ndt->setResolution(ndt_resolution);
      return ndt;
    } else {
      return nullptr;
      /*
      int num_threads = 4;
      std::string nn_search_method = "DIRECT7";
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if(num_threads > 0) {
        ndt->setNumThreads(num_threads);
      }
      ndt->setTransformationEpsilon(0.01);
      ndt->setMaximumIterations(64);
      ndt->setResolution(ndt_resolution);
      if(nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if(nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
      */
    }
  }

  return nullptr;
}

}  // namespace hdl_graph_slam
