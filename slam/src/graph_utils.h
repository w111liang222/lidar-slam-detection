#ifndef __GRAPH_UTILS_H
#define __GRAPH_UTILS_H

#include "py_utils.h"
#include "backend_api.h"

py::array_t<float> pointcloud_align(py::array_t<float> &source_point, py::array_t<float> &target_point, py::array_t<float> &guess);

void set_mapping_ground_constraint(bool enable);
bool get_mapping_ground_constraint();
void set_mapping_constraint(bool loop_closure, bool gravity_constraint);
void set_map_colouration(bool enable);

py::dict get_graph_edges();
py::dict get_graph_meta();
void del_graph_vertex(int id);
void add_graph_edge(py::array_t<float> &prev, int prev_id,
                    py::array_t<float> &next, int next_id,
                    py::array_t<float> relative);
void del_graph_edge(int id);
void set_graph_vertex_fix(int id, bool fix);
py::dict run_graph_optimization();
py::dict run_robust_graph_optimization(std::string mode);

void dump_keyframe(const std::string& directory, uint64_t stamp, int id, py::array_t<float> &points_input, py::array_t<float> &pose_input);
void dump_odometry(const std::string& directory);
void set_export_map_config(double z_min, double z_max, std::string color);
void export_points(py::array_t<float> &points_input, py::array_t<float> &odom_input);
void dump_map_points(std::string file);
py::list dump_graph(const std::string& directory);

/*************** tools for mapping postprocess ***************/
py::list align_pose(double stamp1, double stamp2, py::array_t<double> &estimate1_py, py::array_t<double> &estimate2_py, py::array_t<double> &poses_stamp, py::list &poses_py);
void save_undistortion_cloud(std::string file, py::array_t<float> &points, py::dict &points_attr, py::array_t<double> &poses);
void accumulate_cloud(py::array_t<float> &points, py::dict &points_attr, py::array_t<double> &poses, std::string odometry_type, bool extract_ground);
void save_accumulate_cloud(std::string file, double resolution);
void texture_mesh(std::string mesh_path, std::string cloud_path, std::string output_path);

// offline colouration cloud
void set_colouration_config(py::list &cameras);
void set_map_odometrys(py::array_t<double> &poses);
void colouration_frame(std::string lidar_name, py::dict& points, py::dict &points_attr, py::dict& image_dict, py::dict& image_stream_dict, py::dict& image_param);
void save_render_cloud(std::string file);

#endif //__GRAPH_UTILS_H