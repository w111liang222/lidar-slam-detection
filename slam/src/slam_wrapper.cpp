#include "py_utils.h"
#include "graph_utils.h"

std::unique_ptr<SLAM> slam_ptr(nullptr);

py::list init_slam(const std::string mode,
                   const std::string map_path, const std::string method, py::list& sensor_input,
                   double resolution, float dist_threshold, float degree_threshold, float frame_range) {
  slam_ptr.reset(new SLAM(SLAM::getRunModeType(mode), SLAM::getMappingTypeByName(method)));

  std::vector<std::string> sensors = slam_ptr->setSensors(list_to_vector(sensor_input));
  slam_ptr->setParams(map_path, resolution, dist_threshold, degree_threshold, frame_range);
  return py::cast(sensors);
}

bool setup_slam() {
  return slam_ptr->setup();
}

void deinit_slam() {
  slam_ptr.reset(nullptr);
}

void set_camera_param(py::list &cameras) {
  slam_ptr->setCameraParameter(pylist_to_camera_config(cameras));
}

void set_ins_external_param(double x, double y, double z, double yaw, double pitch, double roll) {
  slam_ptr->setStaticParameter(getTransformFromRPYT(x, y, z, yaw, pitch, roll));
}

void set_imu_external_param(double x, double y, double z, double yaw, double pitch, double roll) {
  slam_ptr->setImuStaticParameter(getTransformFromRPYT(x, y, z, yaw, pitch, roll));
}

void set_ins_config(py::dict& dict) {
  slam_ptr->setInsConfig(pydict_to_ins_config(dict));
  slam_ptr->setInsDimension(pydict_to_ins_dimension(dict));
}

void set_init_pose(double x, double y, double z, double yaw, double pitch, double roll) {
  slam_ptr->setInitPose(getTransformFromRPYT(x, y, z, yaw, pitch, roll));
}

py::list get_estimate_pose(double x0, double y0, double x1, double y1) {
  py::gil_scoped_release release;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  int result = slam_ptr->getEstimatePose(x0, y0, x1, y1, T);
  double x, y, z, yaw, pitch, roll;
  getRPYTfromTransformFrom(T, x, y, z, yaw, pitch, roll);
  std::vector<double> list{x, y, z, roll, pitch, -yaw, result};

  py::gil_scoped_acquire acquire;
  return py::cast(list);
}

void set_destination(bool enable, std::string dest, int port) {
  slam_ptr->setDestination(enable, dest, port);
}

py::dict process(py::dict& points, py::dict &points_attr, py::dict& image_dict, py::dict& image_stream_dict, py::dict& image_param,
                 py::dict& rtk_dict, py::array_t<double>& imu_list, uint64_t timestamp) {
  // convert pydict to pointcloud
  std::map<std::string, PointCloudAttrPtr> clouds;
  pydict_to_cloud(points, points_attr, clouds);

  // convert pydict to cv::Mat (I420 or RGB)
  std::map<std::string, ImageType> images;
  pydict_to_image(image_dict, image_param, images);

  // convert pydict to cv::Mat (JPEG)
  std::map<std::string, cv::Mat> images_stream;
  pydict_to_mat(image_stream_dict, images_stream);

  // convert pydict to RTKType
  RTKType rtk;
  pydict_to_rtk(rtk_dict, rtk);

  // convert numpy to ImuType
  std::vector<ImuType> imu;
  numpy_to_imu(imu_list, imu);

  py::gil_scoped_release release;

  // process frames to estimate the current pose
  PoseType pose;
  slam_ptr->run(timestamp, clouds, images, images_stream, rtk, imu, pose);

  py::gil_scoped_acquire acquire;

  py::dict pose_dict;
  pose_dict["latitude"]    = pose.latitude;
  pose_dict["longitude"]   = pose.longitude;
  pose_dict["altitude"]    = pose.altitude;
  pose_dict["heading"]     = pose.heading;
  pose_dict["pitch"]       = pose.pitch;
  pose_dict["roll"]        = pose.roll;
  pose_dict["Ve"]          = 0;
  pose_dict["Vn"]          = 0;
  pose_dict["Vu"]          = 0;
  pose_dict["Status"]      = pose.status;
  pose_dict["state"]       = pose.state;
  pose_dict["timestamp"]   = pose.timestamp;
  pose_dict["odom_matrix"] = eigen_to_numpy(pose.T);

  py::dict data_dict;
  data_dict["frame_start_timestamp"] = timestamp;
  data_dict["pose"]                  = pose_dict;
  data_dict["slam_valid"]            = true;
  return data_dict;
}

py::dict update_odom() {
  py::list l;
  // flush keyframes
  PointCloudAttrImagePose keyframe;
  while (slam_ptr->getKeyframe(keyframe)) {
    py::dict d;
    d["points"]     = eigen_to_numpy(keyframe.points->cloud);
    d["image"]      = map_to_pydict(keyframe.images_stream);
    d["pose"]       = eigen_to_numpy(keyframe.T.matrix());
    d["stamp"]      = keyframe.points->cloud->header.stamp;
    l.attr("append")(d);
  }

  // graph pose
  std::map<int, Eigen::Matrix4d> odoms;
  graph_update_odom(odoms);

  py::dict dict;
  dict["odoms"]     = map_to_pydict(odoms);
  dict["keyframes"] = l;
  return dict;
}

py::dict get_graph_status() {
  bool loop_detected;
  graph_get_status(loop_detected);

  py::dict dict;
  dict["loop_detected"] = loop_detected;
  return dict;
}

py::array_t<double> get_map_origin() {
  RTKType &pose = slam_ptr->getOrigin();
  MapCoordinateType map_coordinate = slam_ptr->getMapCoordinate();

  std::vector<double> origin(7, 0);
  origin[0] = pose.latitude;
  origin[1] = pose.longitude;
  origin[2] = pose.altitude;
  origin[3] = pose.heading;
  origin[4] = pose.pitch;
  origin[5] = pose.roll;
  origin[6] = static_cast<int>(map_coordinate);
  return py::array_t<double>(py::array::ShapeContainer({1, 7}), origin.data());
}

void set_map_origin(double lat, double lon, double alt, double heading, double pitch, double roll) {
  RTKType rtk;
  rtk.latitude  = lat;
  rtk.longitude = lon;
  rtk.altitude  = alt;
  rtk.heading   = heading;
  rtk.pitch     = pitch;
  rtk.roll      = roll;
  slam_ptr->setOrigin(rtk);
}

// graph interface

py::dict merge_map(const std::string& directory) {
  std::vector<std::shared_ptr<KeyFrame>> frames;
  slam_ptr->mergeMap(directory, frames);
  return keyframe_to_pydict(frames);
}

py::dict get_graph_map() {
  std::vector<std::shared_ptr<KeyFrame>> frames;
  slam_ptr->getGraphMap(frames);
  return keyframe_to_pydict(frames);
}

py::array_t<float> get_color_map() {
  PointCloudRGB::Ptr points(new PointCloudRGB());
  slam_ptr->getColorMap(points);
  return pointcloud_to_numpy(points);
}

PYBIND11_MODULE(slam_wrapper, m) {
  m.doc() = "mapping python interface";
  m.def("init_slam", &init_slam, "init slam",
        py::arg("mode"), py::arg("map_path"), py::arg("method"), py::arg("sensor_input"),
        py::arg("resolution"), py::arg("dist_threshold"), py::arg("degree_threshold"), py::arg("frame_range")
  );
  m.def("setup_slam", &setup_slam, py::call_guard<py::gil_scoped_release>());
  m.def("deinit_slam", &deinit_slam, "deinit slam");
  m.def("set_camera_param", &set_camera_param, "set camera parameters",
        py::arg("cameras")
  );
  m.def("set_ins_external_param", &set_ins_external_param, "set ins external param",
        py::arg("x"), py::arg("y"), py::arg("z"), py::arg("yaw"), py::arg("pitch"), py::arg("roll")
  );
  m.def("set_imu_external_param", &set_imu_external_param, "set imu external param",
        py::arg("x"), py::arg("y"), py::arg("z"), py::arg("yaw"), py::arg("pitch"), py::arg("roll")
  );
  m.def("set_ins_config", &set_ins_config, "set ins config",
        py::arg("dict")
  );
  m.def("set_init_pose", &set_init_pose, "set init pose",
        py::arg("x"), py::arg("y"), py::arg("z"), py::arg("yaw"), py::arg("pitch"), py::arg("roll")
  );
  m.def("get_estimate_pose", &get_estimate_pose, "get estimate pose",
        py::arg("x0"), py::arg("y0"), py::arg("x1"), py::arg("y1")
  );
  m.def("set_destination", &set_destination, "set destination",
        py::arg("enable"), py::arg("dest"), py::arg("port")
  );
  m.def("process", &process, "process",
        py::arg("points"), py::arg("points_attr"), py::arg("image_dict"), py::arg("image_stream_dict"), py::arg("image_param"),
        py::arg("rtk_dict"), py::arg("imu_list"), py::arg("timestamp")
  );
  m.def("update_odom", &update_odom, "update odom");
  m.def("get_graph_status", &get_graph_status, "get graph status");
  m.def("get_map_origin", &get_map_origin, "get map origin");
  m.def("set_map_origin", &set_map_origin, "set map origin",
        py::arg("lat"), py::arg("lon"), py::arg("alt"), py::arg("heading"), py::arg("pitch"), py::arg("roll")
  );
  m.def("get_color_map", &get_color_map, "get color map");

  // graph interface
  m.def("merge_map", &merge_map, "merge map",
        py::arg("directory")
  );
  m.def("get_graph_map", &get_graph_map, "get graph map");
  m.def("get_graph_edges", &get_graph_edges, "get graph edges");

  // graph_utils.cpp
  m.def("pointcloud_align", &pointcloud_align, "pointcloud align",
        py::arg("source_point"), py::arg("target_point"), py::arg("guess"));
  m.def("set_mapping_ground_constraint", &set_mapping_ground_constraint, "set mapping ground constraint",
        py::arg("enable")
  );
  m.def("get_mapping_ground_constraint", &get_mapping_ground_constraint, "get mapping ground constraint");
  m.def("set_mapping_constraint", &set_mapping_constraint, "set mapping constraint",
        py::arg("loop_closure"), py::arg("gravity_constraint")
  );
  m.def("set_map_colouration", &set_map_colouration, "set map colouration",
        py::arg("enable")
  );
  m.def("get_graph_meta", &get_graph_meta, "get graph meta");
  m.def("del_graph_vertex", &del_graph_vertex, "del graph vertex",
        py::arg("id")
  );
  m.def("add_graph_edge", &add_graph_edge, "add graph edge",
        py::arg("prev"), py::arg("prev_id"),
        py::arg("next"), py::arg("next_id"),
        py::arg("relative")
  );
  m.def("del_graph_edge", &del_graph_edge, "del graph edge",
        py::arg("id")
  );
  m.def("set_graph_vertex_fix", &set_graph_vertex_fix, "set graph vertex fix",
        py::arg("id"), py::arg("fix")
  );
  m.def("run_graph_optimization", &run_graph_optimization, "run graph optimization");
  m.def("run_robust_graph_optimization", &run_robust_graph_optimization, "run robust graph optimization",
        py::arg("mode")
  );

  m.def("dump_keyframe", &dump_keyframe, "dump keyframe",
        py::arg("directory"), py::arg("stamp"), py::arg("id"),
        py::arg("points_input"), py::arg("pose_input")
  );
  m.def("dump_odometry", &dump_odometry, "dump odometry",
        py::arg("directory")
  );
  m.def("set_export_map_config", &set_export_map_config, "set export map config",
        py::arg("z_min"), py::arg("z_max"), py::arg("color")
  );
  m.def("export_points", &export_points, "export points",
        py::arg("points_input"), py::arg("odom_input")
  );
  m.def("dump_map_points", &dump_map_points, "dump map points",
        py::arg("file")
  );
  m.def("dump_graph", &dump_graph, "dump graph",
        py::arg("directory")
  );

  // tools for postprocess
  m.def("align_pose", &align_pose, "align pose",
        py::arg("stamp1"), py::arg("stamp2"), py::arg("estimate1_py"), py::arg("estimate2_py"),
        py::arg("poses_stamp"), py::arg("poses_py")
  );
  m.def("save_undistortion_cloud", &save_undistortion_cloud, "save undistortion cloud",
        py::arg("file"), py::arg("points"), py::arg("points_attr"), py::arg("poses")
  );
  m.def("accumulate_cloud", &accumulate_cloud, "accumulate cloud",
        py::arg("points"), py::arg("points_attr"), py::arg("poses"),  py::arg("odometry_type"), py::arg("extract_ground")
  );
  m.def("save_accumulate_cloud", &save_accumulate_cloud, "save accumulate cloud",
        py::arg("file"), py::arg("resolution")
  );
  m.def("texture_mesh", &texture_mesh, "texture mesh",
        py::arg("mesh_path"), py::arg("cloud_path"), py::arg("output_path")
  );

  // offline colouration cloud
  m.def("set_colouration_config", &set_colouration_config, "set colouration config",
        py::arg("cameras")
  );
  m.def("set_map_odometrys", &set_map_odometrys, "set map odometrys",
        py::arg("poses")
  );
  m.def("colouration_frame", &colouration_frame, "colouration frame",
        py::arg("lidar_name"), py::arg("points"), py::arg("points_attr"), py::arg("image_dict"), py::arg("image_stream_dict"), py::arg("image_param")
  );
  m.def("save_render_cloud", &save_render_cloud, "save render cloud",
        py::arg("file")
  );
}