#include "graph_utils.h"

#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

py::array_t<float> pointcloud_align(py::array_t<float> &source_point, py::array_t<float> &target_point, py::array_t<float> &guess) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr source = numpy_to_pointcloud(source_point);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target = numpy_to_pointcloud(target_point);
  Eigen::Matrix4d init_guess = numpy_to_eigen(guess);
  double dist = init_guess(0, 3) * init_guess(0, 3) + \
                init_guess(1, 3) * init_guess(1, 3) + \
                init_guess(2, 3) * init_guess(2, 3);
  dist = std::sqrt(dist);
  if (dist >= 50.0) {
    init_guess(0, 3) = 0;
    init_guess(1, 3) = 0;
    init_guess(2, 3) = 0;
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setTransformationEpsilon(1e-2);
  icp.setMaximumIterations(64);
  icp.setCorrespondenceRandomness(20);

  icp.setInputSource(source);
  icp.setInputTarget(target);
  pcl::PointCloud<pcl::PointXYZI> align;
  icp.align(align, init_guess.cast<float>());
  Eigen::Matrix4f t = icp.getFinalTransformation();

  return eigen_to_numpy(t.cast<double>());
}

void set_mapping_ground_constraint(bool enable) {
  set_ground_constaint(enable);
}

bool get_mapping_ground_constraint() {
  return get_ground_constaint();
}

void set_mapping_loop_closure(bool enable) {
  set_loop_closure(enable);
}

py::dict get_graph_meta() {
  GraphInfo meta;
  graph_get_info(meta);
  py::dict vertex_info_dict, edge_info_dict;
  for(auto &v : meta.vertex) {
    py::dict v_info_dict;
    v_info_dict["fix"] = v.second.fix;
    v_info_dict["edge_num"] = v.second.edge_num;
    vertex_info_dict[std::to_string(v.first).c_str()] = v_info_dict;
  }
  for(auto &e : meta.edge) {
    py::dict e_info_dict;
    e_info_dict["vertex_num"] = e.second.vertex_num;
    edge_info_dict[std::to_string(e.first).c_str()] = e_info_dict;
  }

  py::dict meta_dict;
  meta_dict["vertex"] = vertex_info_dict;
  meta_dict["edge"]   = edge_info_dict;
  return meta_dict;
}

void del_graph_vertex(int id) {
  graph_del_vertex(id);
}

void add_graph_edge(py::array_t<float> &prev, int prev_id,
                    py::array_t<float> &next, int next_id,
                    py::array_t<float> relative) {
  Eigen::Isometry3d relative_pose = Eigen::Isometry3d(numpy_to_eigen(relative));
  graph_add_edge(numpy_to_pointcloud(prev), prev_id, numpy_to_pointcloud(next), next_id, relative_pose);
}

void del_graph_edge(int id) {
  graph_del_edge(id);
}

void set_graph_vertex_fix(int id, bool fix) {
  graph_set_vertex_fix(id, fix);
}

py::dict run_graph_optimization() {
  std::map<int, Eigen::Matrix4d> odoms;
  graph_optimize(odoms);
  return map_to_pydict(odoms);
}

void dump_keyframe(const std::string& directory, uint64_t stamp, int id, py::array_t<float> &points_input, py::array_t<float> &odom_input) {
  PointCloud::Ptr points = numpy_to_pointcloud(points_input, 255.0);
  Eigen::Matrix4d odom = numpy_to_eigen(odom_input);

  py::gil_scoped_release release;
  KeyFrame key_frame(stamp, id, Eigen::Isometry3d(odom), points);
  key_frame.save(directory);
  py::gil_scoped_acquire acquire;
}

static PointCloud::Ptr gMapPoints(new PointCloud());
static PointCloudRGB::Ptr gMapPointsRGB(new PointCloudRGB());
static double gMapZMin = 0, gMapZMax = 0;
void reset_map_points(double z_min, double z_max) {
  gMapZMin = z_min;
  gMapZMax = z_max;
  gMapPoints = PointCloud::Ptr(new PointCloud());
  gMapPointsRGB = PointCloudRGB::Ptr(new PointCloudRGB());
  LOG_INFO("Set export map z min: {}, z max: {}", z_min, z_max);
}

void merge_pcd(py::array_t<float> &points_input, py::array_t<float> &odom_input, bool is_rgb) {
  if (!is_rgb) {
    PointCloud::Ptr points = numpy_to_pointcloud(points_input, 255.0);
    pcl::transformPointCloud(*points, *points, numpy_to_eigen(odom_input));
    for (auto &p : points->points) {
      if (p.z >= gMapZMin && p.z <= gMapZMax) {
        gMapPoints->points.push_back(p);
      }
    }
  } else {
    PointCloudRGB::Ptr points = numpy_to_pointcloud_rgb(points_input);
    pcl::transformPointCloud(*points, *points, numpy_to_eigen(odom_input));
    for (auto &p : points->points) {
      if (p.z >= gMapZMin && p.z <= gMapZMax) {
        gMapPointsRGB->points.push_back(p);
      }
    }
  }
}

void dump_merged_pcd(std::string file) {
  if (gMapPoints->points.size() != 0) {
    gMapPoints->width = static_cast<int>(gMapPoints->points.size());
    gMapPoints->height = 1;
    pcl::io::savePCDFileBinary(file, *gMapPoints);
  } else if (gMapPointsRGB->points.size() != 0) {
    gMapPointsRGB->width = static_cast<int>(gMapPointsRGB->points.size());
    gMapPointsRGB->height = 1;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
    voxelgrid.setLeafSize(0.1, 0.1, 0.1);
    voxelgrid.setInputCloud(gMapPointsRGB);
    voxelgrid.filter(*gMapPointsRGB);
    pcl::io::savePCDFileBinary(file, *gMapPointsRGB);
  }

  gMapPoints = PointCloud::Ptr(new PointCloud());
  gMapPointsRGB = PointCloudRGB::Ptr(new PointCloudRGB());
}

py::list dump_graph(const std::string& directory) {
  return py::cast(graph_save(directory));
}