// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/graph_slam.hpp>

#include <boost/format.hpp>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/edge_xyz_prior.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_gnss.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/robust_kernel_dcs2.hpp>
#include <g2o/robust_kernel_io.hpp>

#include "Logger.h"
#include "SystemUtils.h"

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  // be aware of that cholmod brings GPL dependency
G2O_USE_OPTIMIZATION_LIBRARY(csparse)  // be aware of that csparse brings LGPL unless it is dynamically linked

namespace g2o {
G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
G2O_REGISTER_TYPE(EDGE_SE3_GNSS, EdgeSE3GNSS)
G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_NORMAL, EdgePlanePriorNormal)
G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_DISTANCE, EdgePlanePriorDistance)
G2O_REGISTER_TYPE(EDGE_PLANE_IDENTITY, EdgePlaneIdentity)
G2O_REGISTER_TYPE(EDGE_PLANE_PARALLEL, EdgePlaneParallel)
G2O_REGISTER_TYPE(EDGE_PLANE_PAERPENDICULAR, EdgePlanePerpendicular)
G2O_REGISTER_ROBUST_KERNEL(DCS2, RobustKernelDCS2)
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM(const std::string& solver_type) {
  graph.reset(new g2o::SparseOptimizer());
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  LOG_INFO("construct solver: {}", solver_type);
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if(!graph->solver()) {
    LOG_ERROR("error : failed to allocate solver!!");
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  // std::cout << "done" << std::endl;

  robust_kernel_factory = g2o::RobustKernelFactory::instance();
  max_vertex_id = 0;
  max_edge_id = 0;
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() {
  graph.reset();
}

void GraphSLAM::set_solver(const std::string& solver_type) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  LOG_INFO("construct solver: {}", solver_type);
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if(!graph->solver()) {
    LOG_ERROR("error : failed to allocate solver!!");
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  // std::cout << "done" << std::endl;
}

int GraphSLAM::num_vertices() const {
  return graph->vertices().size();
}
int GraphSLAM::num_edges() const {
  return graph->edges().size();
}

g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(max_vertex_id++);
  vertex->setEstimate(pose);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs, const int &id) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(id);
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz, const int &id) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(id);
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setId(max_edge_id++);
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::del_se3_node(g2o::VertexSE3* n) {
  return graph->removeVertex(n);
}

bool GraphSLAM::del_se3_edge(g2o::EdgeSE3* e) {
  return graph->removeEdge(e);
}

g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setId(max_edge_id++);
  edge->setMeasurement(plane_coeffs);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ* GraphSLAM::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setId(max_edge_id++);
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlanePriorNormal* GraphSLAM::add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorNormal* edge(new g2o::EdgePlanePriorNormal());
  edge->setId(max_edge_id++);
  edge->setMeasurement(normal);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlanePriorDistance* GraphSLAM::add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorDistance* edge(new g2o::EdgePlanePriorDistance());
  edge->setId(max_edge_id++);
  edge->setMeasurement(distance);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
  edge->setId(max_edge_id++);
  edge->setMeasurement(xy);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
  edge->setId(max_edge_id++);
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorVec* GraphSLAM::add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
  Eigen::Matrix<double, 6, 1> m;
  m.head<3>() = direction;
  m.tail<3>() = measurement;

  g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
  edge->setId(max_edge_id++);
  edge->setMeasurement(m);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorQuat* GraphSLAM::add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
  edge->setId(max_edge_id++);
  edge->setMeasurement(quat);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3GNSS* GraphSLAM::add_se3_gnss_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3GNSS* edge(new g2o::EdgeSE3GNSS());
  edge->setId(max_edge_id++);
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeXYZPrior* GraphSLAM::add_xyz_prior_edge(g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::Matrix3d& information_matrix, const int &id) {
  g2o::EdgeXYZPrior* edge(new g2o::EdgeXYZPrior());
  edge->setId(id);
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlane* GraphSLAM::add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) {
  g2o::EdgePlane* edge(new g2o::EdgePlane());
  edge->setId(max_edge_id++);
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlaneIdentity* GraphSLAM::add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) {
  g2o::EdgePlaneIdentity* edge(new g2o::EdgePlaneIdentity());
  edge->setId(max_edge_id++);
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlaneParallel* GraphSLAM::add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information) {
  g2o::EdgePlaneParallel* edge(new g2o::EdgePlaneParallel());
  edge->setId(max_edge_id++);
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlanePerpendicular* GraphSLAM::add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgePlanePerpendicular* edge(new g2o::EdgePlanePerpendicular());
  edge->setId(max_edge_id++);
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

void GraphSLAM::add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size) {
  if(kernel_type == "NONE") {
    return;
  }

  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  if(kernel == nullptr) {
    LOG_WARN("warning : invalid robust kernel type: {}", kernel_type);
    return;
  }

  kernel->setDelta(kernel_size);

  g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
  edge_->setRobustKernel(kernel);
}

int GraphSLAM::optimize(int num_iterations) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
  if(graph->edges().size() < 10) {
    return -1;
  }

  LOG_INFO("--- pose graph optimization ---");
  LOG_INFO("nodes: {}, edges: {}", graph->vertices().size(), graph->edges().size());
  // std::cout << "optimizing... " << std::flush;

  // std::cout << "init" << std::endl;
  auto clock = std::chrono::steady_clock::now();
  graph->initializeOptimization();
  graph->setVerbose(false);

  // std::cout << "chi2" << std::endl;
  double chi2 = graph->chi2();

  // std::cout << "optimize!!" << std::endl;
  // auto t1 = ros::WallTime::now();
  int iterations = graph->optimize(num_iterations);
  auto elapseMs = since(clock).count();

  // auto t2 = ros::WallTime::now();
  // std::cout << "done" << std::endl;
  LOG_INFO("Graph: pose optimization costs {} ms", elapseMs);
  LOG_INFO("iterations: {} / {}", iterations, num_iterations);
  LOG_INFO("chi2: (before) {} -> (after) {}", chi2, graph->chi2());
  // std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

  return iterations;
}

void GraphSLAM::save(const std::string& filename) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  g2o::HyperGraph::VertexSet vertices_to_save;
  for(const auto& v : graph->vertices()) {
    vertices_to_save.insert(v.second);
  }

  std::ofstream ofs(filename);
  graph->saveSubset(ofs, vertices_to_save);

  g2o::save_robust_kernels(filename + ".kernels", graph);
}

bool GraphSLAM::load(const std::string& filename) {
  LOG_INFO("loading pose graph...");
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::ifstream ifs(filename);
  if(!graph->load(ifs)) {
    return false;
  }

  LOG_INFO("nodes  : {}", graph->vertices().size());
  LOG_INFO("edges  : {}", graph->edges().size());

  if(!g2o::load_robust_kernels(filename + ".kernels", graph)) {
    return false;
  }

  return true;
}

}  // namespace hdl_graph_slam
