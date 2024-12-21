#ifndef GLOBAL_ALIGNMENT_HPP
#define GLOBAL_ALIGNMENT_HPP

#include <g2o/core/hyper_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/robust_kernel_io.hpp>

#include <hdl_graph_slam/registrations.hpp>

#include "slam_base.h"
#include "backend_api.h"
#include "Logger.h"

#include "Scancontext/Scancontext.h"
#include "findClique.h"

namespace g2o {
class VertexSE3;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
}  // namespace g2o

const double SC_DIST_THRES = 0.30;
const double ICP_SCORE_THRES = 1.5;
const double PCM_THRES = 1.635; // 95% confidence
const double FITNESS_SCORE_MAX = 25; // 5 m

const std::string consistency_matrix_file = "output/consistency_matrix.clq.mtx";

struct Closures {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Closures>;

  Closures(const std::shared_ptr<KeyFrame>& key1, const std::shared_ptr<KeyFrame>& key2, const Eigen::Matrix4d& relpose, const double& score) : key1(key1), key2(key2), relative_pose(relpose), score(score) {}

public:
  std::shared_ptr<KeyFrame> key1;
  std::shared_ptr<KeyFrame> key2;
  Eigen::Isometry3d relative_pose;
  double score;
};

// Rotation, Translation
Eigen::Matrix<double, 6, 1> Logmap(const Eigen::Matrix4d &matrix) {
    Eigen::AngleAxisd angle_axis(Eigen::Quaterniond(matrix.topLeftCorner<3, 3>()));
    return (Eigen::Matrix<double, 6, 1>() << angle_axis.angle() * angle_axis.axis(), matrix.topRightCorner<3, 1>()).finished();
}

double computeConsistencyError(const Eigen::Isometry3d &z_ai_aj, const Eigen::Isometry3d &z_aj_bl, const Eigen::Isometry3d &z_bl_bk, const Eigen::Isometry3d &z_ai_bk) {
    Eigen::Isometry3d result = z_ai_aj * z_aj_bl * z_bl_bk * z_ai_bk.inverse();
    Eigen::Matrix<double, 6, 1> consistency_error = Logmap(result.matrix());

    // Covariance matrix with usual value (rotation std: 0.1 rad, translation std: 1.0 m)
    const Eigen::MatrixXd FIXED_COVARIANCE =
        (Eigen::MatrixXd(6, 6)  <<   0.01,   0,      0,      0,     0,     0,
                                     0,      0.01,   0,      0,     0,     0,
                                     0,      0,      0.01,   0,     0,     0,
                                     0,      0,      0,      1.0,   0,     0,
                                     0,      0,      0,      0,     1.0,   0,
                                     0,      0,      0,      0,     0,     1.0).finished();

    return std::sqrt(consistency_error.transpose() * FIXED_COVARIANCE.inverse() * consistency_error);
}

Eigen::MatrixXi computePCMMatrix(std::vector<Closures::Ptr> &closures) {
    /*  Consistency loop : aXij + abZjl + bXlk - abZik
     *
     *  *   :   robot poses
     *  |   :   odometry measurements
     *  --  :   interrobot measurements
     *
     *                  abZik
     *        Xai*---------------->Xbk*
     *         |                    ^
     *         |                    |
     *    aXij |                    | bXlk
     *         |                    |
     *         v                    |
     *        Xaj*---------------->Xbl*
     *                  abZjl
     *
     */

    Eigen::MatrixXi PCMMat;
    PCMMat.setZero(closures.size(), closures.size());
    for (int i = 0; i < closures.size(); i++) {
        Eigen::Isometry3d z_ai_bk = closures[i]->relative_pose;
        Eigen::Isometry3d t_ai = closures[i]->key1->mOdom;
        Eigen::Isometry3d t_bk = closures[i]->key2->mOdom;

        for (int j = i + 1; j < closures.size(); j++) {
            Eigen::Isometry3d z_aj_bl = closures[j]->relative_pose;
            Eigen::Isometry3d t_aj = closures[j]->key1->mOdom;
            Eigen::Isometry3d t_bl = closures[j]->key2->mOdom;

            Eigen::Isometry3d z_ai_aj = t_ai.inverse() * t_aj;
            Eigen::Isometry3d z_bl_bk = t_bl.inverse() * t_bk;
            float consistency_error = computeConsistencyError(z_ai_aj, z_aj_bl, z_bl_bk, z_ai_bk);

            if (consistency_error < PCM_THRES) {
                PCMMat(i, j) = 1;
            } else {
                PCMMat(i, j) = 0;
            }
        }
    }
    return PCMMat;
}

void printPCMGraph(Eigen::MatrixXi &pcm_matrix, std::string file_name) {
    // Intialization
    int nb_consistent_measurements = 0;

    // Format edges.
    std::stringstream ss;
    for (int i = 0; i < pcm_matrix.rows(); i++) {
        for (int j = i; j < pcm_matrix.cols(); j++) {
            if (pcm_matrix(i,j) == 1) {
                ss << i + 1 << " " << j + 1 << std::endl;
                nb_consistent_measurements++;
            }
        }
    }

    // Write to file
    std::ofstream output_file;
    output_file.open(file_name);
    output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
    output_file << pcm_matrix.rows() << " " << pcm_matrix.cols() << " " << nb_consistent_measurements << std::endl;
    output_file << ss.str();
    output_file.close();
}

Eigen::Isometry3d estimate_transform(std::vector<Closures::Ptr> &closures) {
    g2o::SparseOptimizer graph;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>());
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    graph.setAlgorithm(solver);

    // find the best closure as the initial transform
    double best_score = FITNESS_SCORE_MAX;
    Closures::Ptr best_closure(nullptr);
    for (int i = 0; i < closures.size(); i++) {
        if (best_score > closures[i]->score) {
            best_score = closures[i]->score;
            best_closure = closures[i];
        }
    }

    // Ta,b
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(0);
    vertex->setEstimate(best_closure->key1->mOdom * best_closure->relative_pose * best_closure->key2->mOdom.inverse());
    graph.addVertex(vertex);

    // add observations
    for (int i = 0; i < closures.size(); i++) {
        Eigen::Isometry3d measurement = closures[i]->key1->mOdom * closures[i]->relative_pose * closures[i]->key2->mOdom.inverse();

        // translation
        Eigen::Matrix3d information_xyz = Eigen::Matrix3d::Identity() * 0.01 / (closures[i]->score + 0.01);
        g2o::EdgeSE3PriorXYZ* edge_xyz(new g2o::EdgeSE3PriorXYZ());
        edge_xyz->setId(i * 2);
        edge_xyz->setMeasurement(measurement.matrix().block<3, 1>(0, 3));
        edge_xyz->setInformation(information_xyz);
        edge_xyz->vertices()[0] = vertex;
        edge_xyz->computeError();
        graph.addEdge(edge_xyz);

        // rotation
        Eigen::Matrix3d information_quat = Eigen::Matrix3d::Identity() * 100.0 / (closures[i]->score + 0.01);
        g2o::EdgeSE3PriorQuat* edge_quat(new g2o::EdgeSE3PriorQuat());
        edge_quat->setId(i * 2 + 1);
        edge_quat->setMeasurement(Eigen::Quaterniond(measurement.matrix().block<3, 3>(0, 0)));
        edge_quat->setInformation(information_quat);
        edge_quat->vertices()[0] = vertex;
        edge_quat->computeError();
        graph.addEdge(edge_quat);
    }

    graph.initializeOptimization();
    graph.setVerbose(false);
    double chi2 = graph.chi2();
    graph.optimize(1024);
    LOG_INFO("chi2: (before) {} -> (after) {}", chi2, graph.chi2());

    return vertex->estimate();
}

bool global_alignment(std::vector<std::shared_ptr<KeyFrame>>& keyframes, std::vector<std::shared_ptr<KeyFrame>>& new_keyframes, bool is_reference_fixed) {
    pcl::Registration<Point, Point>::Ptr registration = hdl_graph_slam::select_registration_method("FAST_VGICP");

    SCManager sc_manager;
    sc_manager.SC_DIST_THRES = SC_DIST_THRES;

    // build SC database
    KeyMat polarcontext_invkeys_mat;
    std::vector<Eigen::MatrixXd> polarcontexts;
    for (size_t i = 0; i < keyframes.size(); i++) {
        polarcontext_invkeys_mat.push_back(eig2stdvec(keyframes[i]->mRingkey));
        polarcontexts.push_back(keyframes[i]->mSc);
    }
    sc_manager.buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts);

    // find loop closure between two maps
    std::vector<Closures::Ptr> closures;
    for (size_t i = 0; i < new_keyframes.size(); i++) {
        std::map<int, SCMatch> matches;

        // find best candidate keyframe in database
        std::vector<std::pair<double, double>> search_trans = {
            {0, 0}, {-4, 0}, {4, 0}, {0, -4}, {0, 4}, {-4, -4}, {-4, 4}, {4, -4}, {4, 4}
        };
        for (auto &t : search_trans) {
            Eigen::MatrixXd sc = sc_manager.makeScancontext(*(new_keyframes[i]->mPoints), t.first, t.second);
            std::vector<float> ringkey = eig2stdvec(sc_manager.makeRingkeyFromScancontext(sc));
            Eigen::MatrixXd sectorkey = sc_manager.makeSectorkeyFromScancontext(sc);

            std::vector<SCMatch> candidates = sc_manager.detectCandidateMatch(sc, ringkey, sectorkey);
            for (auto &match : candidates) {
                if (matches.find(match.sc_idx) == matches.end() || matches[match.sc_idx].sc_dist > match.sc_dist) {
                    matches[match.sc_idx] = match;
                }
            }
        }

        // filter matches by ICP
        std::vector<Closures::Ptr> new_closures;
        for (auto &match : matches) {
            registration->setInputTarget(keyframes[match.first]->mPoints);
            registration->setInputSource(new_keyframes[i]->mPoints);

            Eigen::Matrix4f init_guess = yaw2matrix(-match.second.sc_yaw);
            PointCloud::Ptr aligned(new PointCloud());
            registration->align(*aligned, init_guess);
            if(!registration->hasConverged()) {
                continue;
            }

            double score = registration->getFitnessScore(FITNESS_SCORE_MAX);
            Eigen::Matrix4f relative = registration->getFinalTransformation();
            if (score < ICP_SCORE_THRES) {
                new_closures.push_back(std::make_shared<Closures>(keyframes[match.first], new_keyframes[i], relative.cast<double>(), score));
            }
        }
        closures.insert(closures.end(), new_closures.begin(), new_closures.end());
        LOG_INFO("global alignment, progress: {}/{}, process id: {}, matches: {}/{}", i, new_keyframes.size(), new_keyframes[i]->mId, new_closures.size(), matches.size());
    }

    if (closures.size() <= 0) {
        LOG_ERROR("global alignment: no loop closure found");
        return false;
    }

    // pairwiseConsistencyMaximization
    Eigen::MatrixXi consistency_matrix = computePCMMatrix(closures);
    printPCMGraph(consistency_matrix, consistency_matrix_file);

    FMC::CGraphIO gio;
    gio.readGraph(consistency_matrix_file);
    std::vector<int> max_clique_data;
    int max_clique_size = FMC::maxCliqueHeu(gio, max_clique_data);
    if (max_clique_size <= 3) {
        LOG_ERROR("global alignment: few loop closure found, num: {}", max_clique_size);
        return false;
    }

    std::vector<Closures::Ptr> consistent_closures;
    for (int i = 0; i < max_clique_size; i++) {
        consistent_closures.push_back(closures[max_clique_data[i]]);
    }
    LOG_INFO("global alignment: filter loop closure, {} -> {}", closures.size(), consistent_closures.size());

    // estimate the source -> reference transform
    Eigen::Isometry3d transform_rs = estimate_transform(consistent_closures);
    std::cout << "estimated transform from source to reference: " << std::endl << transform_rs.matrix() << std::endl;

    for (size_t i = 0; i < new_keyframes.size(); i++) {
        new_keyframes[i]->mOdom = transform_rs * new_keyframes[i]->mOdom;
    }
    graph_sync_pose(new_keyframes, SyncDirection::TO_GRAPH);

    // release source map
    for (size_t i = 0; i < new_keyframes.size(); i++) {
        graph_set_vertex_fix(new_keyframes[i]->mId, false);
    }

    // fix the reference map
    if (!is_reference_fixed) {
        std::shared_ptr<KeyFrame> anchor_frame = keyframes[0];
        graph_set_vertex_fix(anchor_frame->mId, true);
    }

    return true;
}

#endif  // GLOBAL_ALIGNMENT_HPP