#ifndef __BACKEND_API__H
#define __BACKEND_API__H

// floor detection nodelet
void init_floor_node();
void deinit_floor_node();
FloorCoeffs enqueue_floor(PointCloud::Ptr &points);

// prefiltering nodelet
void init_filter_node(InitParameter &param);
void deinit_filter_node();
PointCloud::Ptr enqueue_filter(PointCloud::Ptr &points);

// scan matching odometry nodelet
void init_scan_match_node(InitParameter &param);
void deinit_scan_match_node();
Eigen::Matrix4d enqueue_scan_match(PointCloud::Ptr &points, Eigen::Matrix4d& init_guess);

// hdl graph slam nodelet
void init_graph_node(InitParameter &param);
void deinit_graph_node();
void graph_set_origin(RTKType &rtk);
bool enqueue_graph(PointCloudAttrImagePose &frame, PointCloudAttrImagePose &keyframe);
void enqueue_graph_floor(FloorCoeffs& floor_coeffs);
void enqueue_graph_gps(bool rtk_valid, std::shared_ptr<RTKType>& rtk);
void enqueue_graph_imu(const uint64_t &timestamp, Eigen::Vector3d &gyro, Eigen::Vector3d &acc);

Eigen::Isometry3d get_odom2map();
void graph_optimization(bool &is_running);
bool graph_update_odom(std::map<int, Eigen::Matrix4d> &odoms);
void graph_get_status(bool &loop_detected);

void set_ground_constaint(bool enable);
bool get_ground_constaint();
void set_constraint(bool loop_closure, bool gravity_constraint);

inline void init_backend(InitParameter &param) {
    init_floor_node();
    init_filter_node(param);
    init_graph_node(param);
}

inline void deinit_backend() {
    deinit_floor_node();
    deinit_filter_node();
    deinit_graph_node();
}

// map edit interface
bool graph_load(const std::string& directory, std::vector<std::shared_ptr<KeyFrame>> &kfs);
bool graph_merge(const std::string& directory, std::vector<std::shared_ptr<KeyFrame>> &kfs);
std::vector<std::string> graph_save(const std::string& directory);

void graph_add_vertex(int id);
void graph_del_vertex(int id);
void graph_add_edge(const PointCloud::Ptr& prev, int prev_id, const PointCloud::Ptr& next, int next_id, Eigen::Isometry3d &relative_pose, double score = -1.0);
void graph_del_edge(int id);
void graph_get_edges(std::vector<EdgeType> &edges);

void graph_set_vertex_fix(int id, bool fix);
void graph_optimize();
void graph_optimize(std::map<int, Eigen::Matrix4d> &odoms);
void robust_graph_optimize(std::string mode, std::map<int, Eigen::Matrix4d> &odoms);
void graph_get_info(GraphInfo &info);

// below functions are only called by "MapLoader"
enum SyncDirection : int {
  FROM_GRAPH = 0,
  TO_GRAPH = 1,
};
void graph_sync_pose(std::vector<std::shared_ptr<KeyFrame>> &kfs, int direction);
void graph_add_prior_edge(int id, Eigen::Isometry3d &prior, double xyz_stddev, double rot_stddev);
void graph_del_prior_edges();
void graph_update_origin(const RTKType &from_origin, const RTKType &to_origin, std::vector<std::shared_ptr<KeyFrame>> &kfs);

#endif //__BACKEND_API__H