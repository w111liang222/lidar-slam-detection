#include <cmath>
#include "py_utils.h"

py::array_t<float> eigen_to_numpy(const Eigen::Matrix4d &e) {
  std::vector<float> matrix(16, 0);
  matrix[0]  = e(0, 0); matrix[1]  = e(0, 1); matrix[2]  = e(0, 2); matrix[3]  = e(0, 3);
  matrix[4]  = e(1, 0); matrix[5]  = e(1, 1); matrix[6]  = e(1, 2); matrix[7]  = e(1, 3);
  matrix[8]  = e(2, 0); matrix[9]  = e(2, 1); matrix[10] = e(2, 2); matrix[11] = e(2, 3);
  matrix[12] = e(3, 0); matrix[13] = e(3, 1); matrix[14] = e(3, 2); matrix[15] = e(3, 3);
  return py::array_t<float>(py::array::ShapeContainer({4, 4}), matrix.data());
}

py::array_t<float> eigen_to_numpy(const PointCloud::Ptr &p) {
  int num = p->points.size();
  std::vector<float> f(num * 4);
  for (int i = 0; i < num; i++) {
    f[4 * i + 0] = p->points[i].x;
    f[4 * i + 1] = p->points[i].y;
    f[4 * i + 2] = p->points[i].z;
    f[4 * i + 3] = p->points[i].intensity;
  }
  return py::array_t<float>(py::array::ShapeContainer({(long) f.size() / 4, 4}), f.data());
}

py::array_t<char> vector_to_numpy(const std::vector<char> &b) {
  return py::array_t<char>(py::array::ShapeContainer({(long) b.size()}), b.data());
}

py::list vector_to_list(const EdgeType &edge) {
  std::vector<int> c {edge.prev, edge.next};
  return py::cast(c);
}

std::vector<std::string> list_to_vector(py::list &l) {
  std::vector<std::string> s;
  for (py::handle obj : l) {
    s.push_back(obj.cast<std::string>());
  }
  return s;
}

py::dict vector_to_pydict(const std::vector<EdgeType> &edges) {
  py::dict dict;
  for (auto &edge : edges) {
    dict[std::to_string(edge.id).c_str()] = vector_to_list(edge);
  }
  return dict;
}

py::dict map_to_pydict(const std::map<int, Eigen::Matrix4d> &m) {
  py::dict dict;
  for(auto &e : m) {
    dict[std::to_string(e.first).c_str()] = eigen_to_numpy(e.second);
  }
  return dict;
}

py::dict map_to_pydict(const std::map<std::string, cv::Mat> &m) {
  py::dict dict;
  for (auto &e : m) {
    if (e.second.type() == CV_8UC3) {
      dict[e.first.c_str()] = py::array_t<uint8_t>(py::array::ShapeContainer({e.second.rows, e.second.cols, 3}), e.second.data);
    } else {
      dict[e.first.c_str()] = py::array_t<uint8_t>(py::array::ShapeContainer({e.second.rows, e.second.cols}), e.second.data);
    }
  }
  return dict;
}

py::dict map_to_pydict(const std::map<std::string, ImageType> &m) {
  py::dict dict;
  for (auto &e : m) {
    if (e.second.image.type() == CV_8UC3) {
      dict[e.first.c_str()] = py::array_t<uint8_t>(py::array::ShapeContainer({e.second.image.rows, e.second.image.cols, 3}), e.second.image.data);
    } else {
      dict[e.first.c_str()] = py::array_t<uint8_t>(py::array::ShapeContainer({e.second.image.rows, e.second.image.cols}), e.second.image.data);
    }
  }
  return dict;
}

Eigen::Matrix4d numpy_to_eigen(const py::array_t<float> &array) {
  auto ref = array.unchecked<2>();
  Eigen::Matrix4d matrix;
  matrix(0, 0) = ref(0, 0); matrix(0, 1) = ref(0, 1); matrix(0, 2) = ref(0, 2); matrix(0, 3) = ref(0, 3);
  matrix(1, 0) = ref(1, 0); matrix(1, 1) = ref(1, 1); matrix(1, 2) = ref(1, 2); matrix(1, 3) = ref(1, 3);
  matrix(2, 0) = ref(2, 0); matrix(2, 1) = ref(2, 1); matrix(2, 2) = ref(2, 2); matrix(2, 3) = ref(2, 3);
  matrix(3, 0) = ref(3, 0); matrix(3, 1) = ref(3, 1); matrix(3, 2) = ref(3, 2); matrix(3, 3) = ref(3, 3);
  return matrix;
}

Eigen::Matrix4d numpy_to_eigen(const py::array_t<double> &array) {
  auto ref = array.unchecked<2>();
  Eigen::Matrix4d matrix;
  matrix(0, 0) = ref(0, 0); matrix(0, 1) = ref(0, 1); matrix(0, 2) = ref(0, 2); matrix(0, 3) = ref(0, 3);
  matrix(1, 0) = ref(1, 0); matrix(1, 1) = ref(1, 1); matrix(1, 2) = ref(1, 2); matrix(1, 3) = ref(1, 3);
  matrix(2, 0) = ref(2, 0); matrix(2, 1) = ref(2, 1); matrix(2, 2) = ref(2, 2); matrix(2, 3) = ref(2, 3);
  matrix(3, 0) = ref(3, 0); matrix(3, 1) = ref(3, 1); matrix(3, 2) = ref(3, 2); matrix(3, 3) = ref(3, 3);
  return matrix;
}

PointCloud::Ptr numpy_to_pointcloud(py::array_t<float> &array, float multiply) {
  PointCloud::Ptr cloud(new PointCloud());
  auto ref = array.unchecked<2>();
  size_t num = ref.shape(0);
  cloud->width = static_cast<int>(num);
  cloud->height = 1;
  cloud->points.resize(num);
  for (size_t i = 0; i < num; i++) {
    cloud->points[i].x         = ref(i, 0);
    cloud->points[i].y         = ref(i, 1);
    cloud->points[i].z         = ref(i, 2);
    cloud->points[i].intensity = ref(i, 3) * multiply;
  }
  return cloud;
}

py::array_t<float> pointcloud_to_numpy(const PointCloudRGB::Ptr &p) {
  int num = p->points.size();
  std::vector<float> f(num * 4);
  for (int i = 0; i < num; i++) {
    f[4 * i + 0] = p->points[i].x;
    f[4 * i + 1] = p->points[i].y;
    f[4 * i + 2] = p->points[i].z;
    std::uint32_t rgb = ((std::uint32_t)(p->points[i].r) << 16 | (std::uint32_t)(p->points[i].g) << 8 | (std::uint32_t)(p->points[i].b));
    f[4 * i + 3] = *reinterpret_cast<float*>(&rgb);
  }
  return py::array_t<float>(py::array::ShapeContainer({(long) f.size() / 4, 4}), f.data());
}

PointCloudRGB::Ptr numpy_to_pointcloud_rgb(py::array_t<float> &array) {
  PointCloudRGB::Ptr cloud(new PointCloudRGB());
  auto ref = array.unchecked<2>();
  size_t num = ref.shape(0);
  cloud->width = static_cast<int>(num);
  cloud->height = 1;
  cloud->points.resize(num);
  for (size_t i = 0; i < num; i++) {
    cloud->points[i].x = ref(i, 0);
    cloud->points[i].y = ref(i, 1);
    cloud->points[i].z = ref(i, 2);
    cloud->points[i].b = ref(i, 3);
    cloud->points[i].g = ref(i, 4);
    cloud->points[i].r = ref(i, 5);
  }
  return cloud;
}

void numpy_to_pointcloud(const py::array_t<float> &points, const py::array_t<float> &points_attr, PointCloudAttrPtr &cloud) {
  auto ref = points.unchecked<2>();
  auto ref_attr = points_attr.unchecked<2>();
  size_t num = ref.shape(0);

  PointCloud::Ptr &pointcloud = cloud->cloud;
  std::vector<PointAttr> &pointcloud_attr = cloud->attr;

  pointcloud->width = static_cast<int>(num);
  pointcloud->height = 1;
  pointcloud->points.resize(num);
  pointcloud_attr.resize(num);
  for (size_t i = 0; i < num; i++) {
    pointcloud->points[i].x         = ref(i, 0);
    pointcloud->points[i].y         = ref(i, 1);
    pointcloud->points[i].z         = ref(i, 2);
    pointcloud->points[i].intensity = ref(i, 3);
    pointcloud_attr[i].stamp        = ref_attr(i, 0);
    pointcloud_attr[i].id           = ref_attr(i, 1);
  }
}

void pydict_to_cloud(py::dict& points, py::dict& points_attr, std::map<std::string, PointCloudAttrPtr> &clouds) {
  for (auto item : points) {
    std::string name = py::cast<std::string>(item.first);
    py::dict attr_dict = py::cast<py::dict>(points_attr[name.c_str()]);

    PointCloudAttrPtr cloud(new PointCloudAttr());
    numpy_to_pointcloud(py::cast<py::array>(item.second), py::cast<py::array>(attr_dict["points_attr"]), cloud);
    cloud->cloud->header.stamp = py::cast<uint64_t>(attr_dict["timestamp"]);
    clouds[name] = cloud;
  }
}

void pydict_to_mat(py::dict& image_dict, std::map<std::string, cv::Mat> &images) {
  for (auto item : image_dict) {
    std::string name = py::cast<std::string>(item.first);
    py::array ar = py::cast<py::array>(item.second);
    auto ndim = ar.ndim();
    auto shape = ar.shape();
    if (ndim == 2) {
      images[name] = cv::Mat(shape[0], shape[1], CV_8UC1);
    } else {
      images[name] = cv::Mat(shape[0], shape[1], CV_8UC3);
    }
    memcpy(images[name].data, ar.data(), ar.nbytes());
  }
}

void pydict_to_image(py::dict& image_dict, py::dict& image_param, std::map<std::string, ImageType> &images) {
  for (auto item : image_dict) {
    std::string name = py::cast<std::string>(item.first);
    py::dict param = py::cast<py::dict>(image_param[name.c_str()]);
    py::array ar = py::cast<py::array>(item.second);
    auto ndim = ar.ndim();
    auto shape = ar.shape();
    if (ndim == 2) {
      images[name] = ImageType(cv::Mat(shape[0], shape[1], CV_8UC1), py::cast<uint64_t>(param["timestamp"]));
    } else {
      images[name] = ImageType(cv::Mat(shape[0], shape[1], CV_8UC3), py::cast<uint64_t>(param["timestamp"]));
    }
    memcpy(images[name].image.data, ar.data(), ar.nbytes());
  }
}

void pydict_to_rtk(py::dict& rtk_dict, RTKType &rtk) {
  rtk.timestamp = py::cast<uint64_t>(rtk_dict["timestamp"]);
  rtk.longitude = py::cast<double>(rtk_dict["longitude"]);
  rtk.latitude  = py::cast<double>(rtk_dict["latitude"]);
  rtk.altitude  = py::cast<double>(rtk_dict["altitude"]);

  rtk.heading   = py::cast<double>(rtk_dict["heading"]);
  rtk.pitch     = py::cast<double>(rtk_dict["pitch"]);
  rtk.roll      = py::cast<double>(rtk_dict["roll"]);

  rtk.gyro_x    = py::cast<double>(rtk_dict["gyro_x"]);
  rtk.gyro_y    = py::cast<double>(rtk_dict["gyro_y"]);
  rtk.gyro_z    = py::cast<double>(rtk_dict["gyro_z"]);

  rtk.acc_x     = py::cast<double>(rtk_dict["acc_x"]);
  rtk.acc_y     = py::cast<double>(rtk_dict["acc_y"]);
  rtk.acc_z     = py::cast<double>(rtk_dict["acc_z"]);

  rtk.Ve        = py::cast<double>(rtk_dict["Ve"]);
  rtk.Vn        = py::cast<double>(rtk_dict["Vn"]);
  rtk.Vu        = py::cast<double>(rtk_dict["Vu"]);
  rtk.status    = py::cast<int>(rtk_dict["Status"]);
  rtk.sensor    = py::cast<std::string>(rtk_dict["Sensor"]);
  if (rtk_dict.contains("state")) {
    rtk.state = py::cast<std::string>(rtk_dict["state"]);
  } else {
    rtk.state = "Unknown";
  }
}

void numpy_to_imu(py::array_t<double> &imu_list, std::vector<ImuType> &imu) {
  auto ref = imu_list.unchecked<2>();
  size_t num = ref.shape(0);
  imu.resize(num);
  for (size_t i = 0; i < num; i++) {
    imu[i].gyr = Eigen::Vector3d(ref(i, 1) / 180.0 * M_PI,
                                 ref(i, 2) / 180.0 * M_PI,
                                 ref(i, 3) / 180.0 * M_PI);
    imu[i].acc = Eigen::Vector3d(ref(i, 4) * 9.81,
                                 ref(i, 5) * 9.81,
                                 ref(i, 6) * 9.81);
    imu[i].rot = Eigen::Quaterniond::Identity();
    imu[i].stamp = ref(i, 0) / 1000000.0;
  }
}

void numpy_to_odometry(py::array_t<double> &poses, std::vector<PoseType> &odometrys) {
  auto pose_ref = poses.unchecked<2>();
  size_t pose_num = pose_ref.shape(0);
  for (size_t i = 0; i < pose_num; i++) {
    PoseType pose;
    pose.timestamp = uint64_t(pose_ref(i, 0));
    pose.T.block<3, 1>(0, 3) = Eigen::Vector3d(pose_ref(i, 1), pose_ref(i, 2), pose_ref(i, 3));
    pose.T.block<3, 3>(0, 0) = Eigen::Quaterniond(pose_ref(i, 7), pose_ref(i, 4), pose_ref(i, 5), pose_ref(i, 6)).toRotationMatrix();
    odometrys.emplace_back(pose);
  }
}

py::dict keyframe_to_pydict(std::vector<std::shared_ptr<KeyFrame>> &frames) {
  py::dict points, images, poses, stamps;
  for (size_t i = 0; i < frames.size(); i++) {
    std::string id = std::to_string(frames[i]->mId);
    points[id.c_str()] = eigen_to_numpy(frames[i]->mPoints);
    poses[id.c_str()]  = eigen_to_numpy(frames[i]->mOdom.matrix());
    stamps[id.c_str()] = frames[i]->mPoints->header.stamp;

    py::dict image;
    for(auto &im : frames[i]->mImages) {
      image[im.first.c_str()] = vector_to_numpy(im.second);
    }
    images[id.c_str()] = image;
  }

  py::dict dict;
  dict["points"] = points;
  dict["images"] = images;
  dict["poses"]  = poses;
  dict["stamps"] = stamps;
  return dict;
}

std::map<int, InsConfig> pydict_to_ins_config(py::dict& dict) {
  std::map<int, InsConfig> config;
  std::vector<std::string> status_names{"ins_normal", "ins_float", "ins_fix"};
  int priority = 0;
  for(auto &name : status_names) {
    bool use = py::cast<bool>(dict[name.c_str()]["use"]);
    int status =  py::cast<int>(dict[name.c_str()]["status"]);
    double stable_time =  py::cast<double>(dict[name.c_str()]["stable_time"]);
    double precision =  py::cast<double>(dict[name.c_str()]["precision"]);
    if (use) {
      config[priority] = InsConfig();
      config[priority].status_name = name;
      config[priority].status      = status;
      config[priority].priority    = priority;
      config[priority].stable_time = stable_time;
      config[priority].precision   = precision;
      priority++;
    }
  }
  return config;
}

int pydict_to_ins_dimension(py::dict& dict) {
  std::string dim = py::cast<std::string>(dict["ins_type"]);
  if (dim.compare("2D") == 0) {
    return 2;
  } else if (dim.compare("3D") == 0) {
    return 3;
  } else if (dim.compare("6D") == 0) {
    return 6;
  }
  return 0;
}

std::map<std::string, CamParamType> pylist_to_camera_config(py::list& cameras) {
  std::map<std::string, CamParamType> cameraParam;
  for (py::handle obj : cameras) {
    py::dict camera = obj.cast<py::dict>();
    std::string name = py::cast<std::string>(camera["name"]);
    py::list intrinsic = py::cast<py::list>(camera["intrinsic_parameters"]);
    py::list extrinsic = py::cast<py::list>(camera["extrinsic_parameters"]);

    // K
    cameraParam[name] = CamParamType();

    cameraParam[name].K.at<float>(0, 0) = py::cast<float>(intrinsic[0]);
    cameraParam[name].K.at<float>(0, 1) = 0.0f;
    cameraParam[name].K.at<float>(0, 2) = py::cast<float>(intrinsic[2]);

    cameraParam[name].K.at<float>(1, 0) = 0.0f;
    cameraParam[name].K.at<float>(1, 1) = py::cast<float>(intrinsic[1]);
    cameraParam[name].K.at<float>(1, 2) = py::cast<float>(intrinsic[3]);

    cameraParam[name].K.at<float>(2, 0) = 0.0f;
    cameraParam[name].K.at<float>(2, 1) = 0.0f;
    cameraParam[name].K.at<float>(2, 2) = 1.0f;

    // extinsic
    cameraParam[name].staticTrans = getTransformFromRPYT(py::cast<double>(extrinsic[0]), py::cast<double>(extrinsic[1]), py::cast<double>(extrinsic[2]),
                                                         py::cast<double>(extrinsic[5]), py::cast<double>(extrinsic[4]), py::cast<double>(extrinsic[3]));
  }

  return cameraParam;
}