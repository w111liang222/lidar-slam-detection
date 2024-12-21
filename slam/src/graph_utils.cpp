#include "graph_utils.h"

#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <boost/algorithm/string.hpp>

#include "localization.h"

extern std::unique_ptr<SLAM> slam_ptr;

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

void set_mapping_constraint(bool loop_closure, bool gravity_constraint) {
  set_constraint(loop_closure, gravity_constraint);
}

void set_map_colouration(bool enable) {
  Locate::Localization::setMapRender(enable);
}

py::dict get_graph_edges() {
  std::vector<EdgeType> edges;
  graph_get_edges(edges);
  return vector_to_pydict(edges);
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

py::dict run_robust_graph_optimization(std::string mode) {
  std::map<int, Eigen::Matrix4d> odoms;
  robust_graph_optimize(mode, odoms);
  return map_to_pydict(odoms);
}

void dump_keyframe(const std::string& directory, uint64_t stamp, int id, py::array_t<float> &points_input, py::array_t<float> &pose_input) {
  PointCloud::Ptr points = numpy_to_pointcloud(points_input, 255.0);
  Eigen::Matrix4d pose = numpy_to_eigen(pose_input);

  py::gil_scoped_release release;
  KeyFrame key_frame(stamp, id, Eigen::Isometry3d(pose), points);
  key_frame.save(directory);
  py::gil_scoped_acquire acquire;
}

void dump_odometry(const std::string& directory) {
  std::vector<PoseType> odometrys;
  slam_ptr->getOdometrys(odometrys);
  std::ofstream ofs(directory + "/odometrys.txt");
  for (auto &odometry : odometrys) {
    const double stamp = odometry.timestamp / 1000000.0;
    const Eigen::Vector3d t = odometry.T.block<3, 1>(0, 3);
    const Eigen::Quaterniond q(odometry.T.block<3, 3>(0, 0));

    ofs << std::fixed << std::setprecision(6) << stamp << " " << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  ofs.close();
}

struct MapConfig{
  MapConfig() {
    points = PointCloud::Ptr(new PointCloud());
    points_color = PointCloudRGB::Ptr(new PointCloudRGB());
  }
  double z_min;
  double z_max;
  std::string color;
  PointCloud::Ptr points;
  PointCloudRGB::Ptr points_color;
};
static MapConfig g_map_config;

void set_export_map_config(double z_min, double z_max, std::string color) {
  g_map_config = MapConfig();
  g_map_config.z_min = z_min;
  g_map_config.z_max = z_max;
  g_map_config.color = color;
  LOG_INFO("set export map z min: {}, z max: {}, color: {}", z_min, z_max, color);
}

void export_points(py::array_t<float> &points_input, py::array_t<float> &odom_input) {
  if (g_map_config.color.compare("rgb") != 0) {
    PointCloud::Ptr points = numpy_to_pointcloud(points_input, 255.0);
    pcl::transformPointCloud(*points, *points, numpy_to_eigen(odom_input));
    for (auto &p : points->points) {
      if (p.z >= g_map_config.z_min && p.z <= g_map_config.z_max) {
        g_map_config.points->points.push_back(p);
      }
    }
  }
}

void dump_map_points(std::string file) {
  if (g_map_config.color.compare("rgb") != 0) {
    if (g_map_config.points->points.size() <= 0) {
      return;
    }

    g_map_config.points->width = static_cast<int>(g_map_config.points->points.size());
    g_map_config.points->height = 1;
    pcl::io::savePCDFileBinary(file, *g_map_config.points);
  } else {
    PointCloudRGB::Ptr points(new PointCloudRGB());
    slam_ptr->getColorMap(points);
    for (auto &p : points->points) {
      if (p.z >= g_map_config.z_min && p.z <= g_map_config.z_max) {
        g_map_config.points_color->points.push_back(p);
      }
    }
    if (g_map_config.points_color->points.size() <= 0) {
      return;
    }

    g_map_config.points_color->width = static_cast<int>(g_map_config.points_color->points.size());
    g_map_config.points_color->height = 1;
    pcl::io::savePCDFileBinary(file, *g_map_config.points_color);
  }

  g_map_config = MapConfig();
}

py::list dump_graph(const std::string& directory) {
  return py::cast(graph_save(directory));
}

/*************** tools for mapping postprocess ***************/

template <typename T>
py::list to_list(T t)
{
  return py::cast(t);
}

py::array_t<double> eigend_to_numpy(const Eigen::Matrix4d &e) {
  std::vector<double> matrix(16, 0);
  matrix[0]  = e(0, 0); matrix[1]  = e(0, 1); matrix[2]  = e(0, 2); matrix[3]  = e(0, 3);
  matrix[4]  = e(1, 0); matrix[5]  = e(1, 1); matrix[6]  = e(1, 2); matrix[7]  = e(1, 3);
  matrix[8]  = e(2, 0); matrix[9]  = e(2, 1); matrix[10] = e(2, 2); matrix[11] = e(2, 3);
  matrix[12] = e(3, 0); matrix[13] = e(3, 1); matrix[14] = e(3, 2); matrix[15] = e(3, 3);
  return py::array_t<double>(py::array::ShapeContainer({4, 4}), matrix.data());
}

py::list align_pose(double stamp1, double stamp2, py::array_t<double> &estimate1_py, py::array_t<double> &estimate2_py, py::array_t<double> &poses_stamp, py::list &poses_py) {
  std::vector<double> pose_stamp;
  std::vector<Eigen::Matrix4d> poses;

  auto poses_stamp_ref = poses_stamp.unchecked<1>();
  for (size_t i = 0; i < poses_stamp_ref.shape(0); i++) {
    pose_stamp.push_back(poses_stamp_ref(i));
  }

  for (auto pose : poses_py) {
    poses.push_back(numpy_to_eigen(py::cast<py::array_t<double>>(pose)));
  }

  for (int i = poses.size() - 1; i >= 0; i--) {
    poses[i] = poses[0].inverse() * poses[i];
  }

  Eigen::Matrix4d estimate1 = numpy_to_eigen(estimate1_py);
  Eigen::Matrix4d estimate2 = numpy_to_eigen(estimate2_py);
  Eigen::Matrix4d delta_odom = estimate1.inverse() * estimate2;
  Eigen::Matrix4d delta_delta_odom = poses.back().inverse() * delta_odom;
  Eigen::Vector3d delta_translation = delta_delta_odom.block<3, 1>(0, 3);
  Eigen::Quaterniond delta_rotation(delta_delta_odom.block<3, 3>(0, 0));
  Eigen::AngleAxisd angle_axis(delta_rotation);

  double duration = stamp2 - stamp1;
  for (int i = 0; i < poses.size(); i++) {
    double t_diff_ratio = (pose_stamp[i] - pose_stamp[0]) / duration;
    Eigen::Matrix<double, 6, 1> log_vector = t_diff_ratio * (Eigen::Matrix<double, 6, 1>() << delta_translation, angle_axis.angle() * angle_axis.axis()).finished();
    constexpr double kEpsilon = 1e-8;
    const float norm = log_vector.tail<3>().norm();
    Eigen::Matrix4d plus_odom = Eigen::Matrix4d::Identity();
    if (norm < kEpsilon) {
        Eigen::Vector3d new_translation = log_vector.head<3>();
        Eigen::Quaterniond new_rotation(Eigen::Quaterniond::Identity());
        plus_odom.block<3, 1>(0, 3) = new_translation;
        plus_odom.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
    } else {
        Eigen::Vector3d new_translation = log_vector.head<3>();
        Eigen::Quaterniond new_rotation(Eigen::AngleAxisd(norm,  log_vector.tail<3>() / norm));
        plus_odom.block<3, 1>(0, 3) = new_translation;
        plus_odom.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
    }
    poses[i] = poses[i] * plus_odom;
    poses[i] = estimate1 * poses[i];
  }

  std::vector<py::array_t<double>> align_poses;
  for (int i = 0; i < poses.size(); i++) {
    align_poses.push_back(eigend_to_numpy(poses[i]));
  }
  return to_list(align_poses);
}


PointCloud::Ptr plane_clip(const PointCloud::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) {
    pcl::PlaneClipper3D<Point> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::ExtractIndices<Point> extract;
    PointCloud::Ptr dst_cloud(new PointCloud());
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);
    return dst_cloud;
}

PointCloud::Ptr normal_filtering(const PointCloud::Ptr& cloud) {
    const double sensor_height = 0.0;
    const double normal_filter_thresh = 20.0;
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);

    PointCloud::Ptr filtered(new PointCloud());
    filtered->reserve(cloud->size());

    for(int i = 0; i < cloud->size(); i++) {
      float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
      if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
        filtered->push_back(cloud->at(i));
      }
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
}

boost::optional<Eigen::Vector4f> detect_ground(const PointCloud::Ptr& cloud) {
    const int    floor_pts_thresh       = 1024;
    const double height_clip_range_low  = 1.5;
    const double height_clip_range_high = 1.5;
    const double floor_normal_thresh    = 10.0;
    // filtering before RANSAC (height and normal filtering)
    PointCloud::Ptr filtered(new PointCloud());
    filtered = cloud;
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f,  height_clip_range_low), false);
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, -height_clip_range_high), true);
    filtered = normal_filtering(filtered);

    // too few points for RANSAC
    if(filtered->size() < floor_pts_thresh) {
      return boost::none;
    }

    // RANSAC
    pcl::SampleConsensusModelPlane<Point>::Ptr model_p(new pcl::SampleConsensusModelPlane<Point>(filtered));
    pcl::RandomSampleConsensus<Point> ransac(model_p);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < floor_pts_thresh) {
      return boost::none;
    }

    // verticality check of the detected floor's normal
    Eigen::Vector4f reference = Eigen::Vector4f::UnitZ();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    double dot = coeffs.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
      // the normal is not vertical
      return boost::none;
    }

    // make the normal upward
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.filter(*cloud);
    return Eigen::Vector4f(coeffs);
}

PointCloudAttrPtr undistortion_cloud(py::array_t<float> &points, py::dict &points_attr, std::vector<PoseType> odometrys) {
  // convert cloud
  PointCloudAttrPtr cloud(new PointCloudAttr());
  numpy_to_pointcloud(points, py::cast<py::array>(points_attr["points_attr"]), cloud);
  cloud->cloud->header.stamp = py::cast<uint64_t>(points_attr["timestamp"]);

  // undistortion pointcloud
  for (int i = odometrys.size() - 1; i >= 0; i--) {
    odometrys[i].T = odometrys[0].T.inverse() * odometrys[i].T;
  }
  undistortPoints(odometrys, cloud);
  return cloud;
}

void save_undistortion_cloud(std::string file, py::array_t<float> &points, py::dict &points_attr, py::array_t<double> &poses) {
  // convert odometrys
  std::vector<PoseType> odometrys;
  numpy_to_odometry(poses, odometrys);

  // undistortion
  PointCloudAttrPtr cloud = undistortion_cloud(points, points_attr, odometrys);

  // save
  pcl::io::savePCDFileBinary(file, *cloud->cloud);
}

static PointCloud::Ptr g_accumulate_cloud(new PointCloud());

void accumulate_cloud(py::array_t<float> &points, py::dict &points_attr, py::array_t<double> &poses, std::string odometry_type, bool extract_ground) {
  // convert odometrys
  std::vector<PoseType> odometrys;
  if (odometry_type.compare("TUM") == 0) {
    numpy_to_odometry(poses, odometrys);
  }

  // undistortion
  PointCloudAttrPtr cloud = undistortion_cloud(points, points_attr, odometrys);
  if (extract_ground && !detect_ground(cloud->cloud)) {
    return;
  }

  // transform to world coordinate
  PointCloud::Ptr transformed_cloud(new PointCloud());
  pcl::transformPointCloud(*cloud->cloud, *transformed_cloud, odometrys[0].T);
  *g_accumulate_cloud += *(transformed_cloud);
}

void save_accumulate_cloud(std::string file, double resolution) {
  if (g_accumulate_cloud->points.size() <= 0) {
    return;
  }

  if (resolution > 0.0) {
    pcl::VoxelGrid<Point> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(g_accumulate_cloud);
    voxelgrid.filter(*g_accumulate_cloud);
  }

  pcl::io::savePCDFileBinary(file, *g_accumulate_cloud);

  g_accumulate_cloud = PointCloud::Ptr(new PointCloud());
  std::cout << "save dense map to: " << file << std::endl;
}

void texture_mesh(std::string mesh_path, std::string cloud_path, std::string output_path) {
  std::cout << "Performaning the mesh texturing..." << std::endl;
  std::cout << "Build Kd tree, please wait..." << std::endl;
  pcl::PCDReader reader;
  PointCloudRGB::Ptr cloud(new PointCloudRGB());
  reader.read(cloud_path, *cloud);

  pcl::KdTreeFLANN<PointRGB> kdtree;
  kdtree.setInputCloud(cloud);
  std::cout << "Build Kd tree finish !" << std::endl;

  std::cout << "Loading mesh to PCL polygon, please wait...." << std::endl;
  pcl::PolygonMesh mesh_obj;
  pcl::io::loadOBJFile(mesh_path, mesh_obj);
  std::cout << "Loading mesh finish" << std::endl;

  PointCloudRGB rgb_pointcloud;
  pcl::fromPCLPointCloud2(mesh_obj.cloud, rgb_pointcloud);

  const int smooth_factor = 3;
  std::vector<int>      pointIdxNKNSearch(smooth_factor);
  std::vector<float>    pointNKNSquaredDistance(smooth_factor);
  for (int i = 0; i < rgb_pointcloud.points.size(); ++i) {
    uint8_t  r = 0;
    uint8_t  g = 0;
    uint8_t  b = 0;
    int      red = 0;
    int      green = 0;
    int      blue = 0;
    if (kdtree.nearestKSearch(rgb_pointcloud.points[i], smooth_factor, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      for (int j = 0; j < pointIdxNKNSearch.size(); ++j) {
        r = cloud->points[pointIdxNKNSearch[j]].r;
        g = cloud->points[pointIdxNKNSearch[j]].g;
        b = cloud->points[pointIdxNKNSearch[j]].b;
        red += int(r);
        green += int(g);
        blue += int(b);
      }
    }

    rgb_pointcloud.points[i].r = int(red / pointIdxNKNSearch.size());
    rgb_pointcloud.points[i].g = int(green / pointIdxNKNSearch.size());
    rgb_pointcloud.points[i].b = int(blue / pointIdxNKNSearch.size());
    rgb_pointcloud.points[i].a = 255;
    if (i % 10000 == 0) {
      printf( "\33[2K\rTexturing mesh [%lu%%] ...", i * 100 / (rgb_pointcloud.points.size() - 1));
    }
  }

  printf( "\33[2K\rTexturing mesh [100%%] \r\n" );
  pcl::toPCLPointCloud2(rgb_pointcloud, mesh_obj.cloud);
  pcl::io::savePLYFileBinary(output_path + "/texture_mesh.ply", mesh_obj);
}

// offline colouration cloud
struct MapColourationContext {
  int odometry_idx;
  std::vector<PoseType> odometrys;
  std::unique_ptr<Locate::MapLoader> map;
  std::map<std::string, std::shared_ptr<CameraModel>> cameras;
  std::deque<PointCloud::Ptr> frame_clouds;
};

static MapColourationContext coloration_context;

bool get_stamp_pose(uint64_t timestamp, int start_idx, int &out_idx, PoseType &pose) {
  if (coloration_context.odometrys.size() <= 0) {
    return false;
  }
  if (timestamp < coloration_context.odometrys.front().timestamp) {
    return false;
  }
  if (timestamp > coloration_context.odometrys.back().timestamp) {
    return false;
  }

  int first_idx = start_idx;
  int second_idx = start_idx;
  for(; second_idx != coloration_context.odometrys.size(); second_idx++) {
    auto dt = (coloration_context.odometrys[first_idx].timestamp - double(timestamp)) / 1000000.0;
    auto dt2 = (coloration_context.odometrys[second_idx].timestamp - double(timestamp)) / 1000000.0;
    if(dt <= 0 && dt2 >= 0) {
      break;
    }
    first_idx = second_idx;
  }
  if (first_idx == second_idx || second_idx == coloration_context.odometrys.size()) {
    return false;
  }

  out_idx = first_idx;
  double t_diff_ratio = (double(timestamp) - coloration_context.odometrys[first_idx].timestamp) / double(coloration_context.odometrys[second_idx].timestamp - coloration_context.odometrys[first_idx].timestamp);
  pose.timestamp = timestamp;
  pose.T = interpolateTransform(coloration_context.odometrys[first_idx].T, coloration_context.odometrys[second_idx].T, t_diff_ratio);
  return true;
}

void set_colouration_config(py::list &cameras) {
  std::map<std::string, CamParamType> camera_param = pylist_to_camera_config(cameras);

  Eigen::Matrix4d static_trans = Eigen::Matrix4d::Identity();
  coloration_context.map.reset(new Locate::MapLoader());
  coloration_context.map->setMapRender(true);
  coloration_context.map->setParameter(static_trans, camera_param);

  for (auto &it : camera_param) {
    coloration_context.cameras[it.first] = std::make_shared<CameraModel>(it.second);
  }
}

void set_map_odometrys(py::array_t<double> &poses) {
  coloration_context.odometry_idx = 0;
  numpy_to_odometry(poses, coloration_context.odometrys);
}

void colouration_frame(std::string lidar_name, py::dict& points, py::dict &points_attr, py::dict& image_dict, py::dict& image_stream_dict, py::dict& image_param) {
  // convert pydict to pointcloud
  std::map<std::string, PointCloudAttrPtr> clouds;
  pydict_to_cloud(points, points_attr, clouds);

  // convert pydict to cv::Mat
  std::map<std::string, ImageType> images;
  pydict_to_image(image_dict, image_param, images);

  // current frame
  PointCloudAttrPtr frame = clouds[lidar_name];

  // get lidar start/end poses
  int start_odometry_idx, end_odometry_idx;
  PoseType start_stamp_pose, end_stamp_pose;
  if (get_stamp_pose(frame->cloud->header.stamp, coloration_context.odometry_idx, start_odometry_idx, start_stamp_pose) == false) {
    return;
  }
  if (get_stamp_pose(frame->cloud->header.stamp + 100000ULL, coloration_context.odometry_idx, end_odometry_idx, end_stamp_pose) == false) {
    return;
  }
  coloration_context.odometry_idx = start_odometry_idx; // update search odometry idx at start scan stamp

  // undistortion pointcloud
  std::vector<PoseType> odometrys;
  odometrys.emplace_back(start_stamp_pose);
  for (int i = start_odometry_idx + 1; i < end_odometry_idx; i++) {
    odometrys.emplace_back(coloration_context.odometrys[i]);
  }
  odometrys.emplace_back(end_stamp_pose);
  for (int i = odometrys.size() - 1; i >= 0; i--) {
    odometrys[i].T = odometrys[0].T.inverse() * odometrys[i].T;
  }
  undistortPoints(odometrys, frame);

  // get camera poses
  std::map<std::string, ImageType> render_images;
  for (auto &im : images) {
    int out_idx = 0;
    PoseType image_pose;
    if (get_stamp_pose(im.second.stamp, coloration_context.odometry_idx, out_idx, image_pose)) {
      im.second.T = image_pose.T;
      render_images[im.first] = im.second;
    }
  }

  // render map
  coloration_context.map->render(start_stamp_pose.T, frame, render_images);
}

void save_render_cloud(std::string file) {
  PointCloudRGB::Ptr points(new PointCloudRGB());
  coloration_context.map->getColorMap(points);
  coloration_context.map.reset(nullptr);

  points->width = static_cast<int>(points->points.size());
  points->height = 1;
  pcl::io::savePCDFileBinary(file, *points);
}
