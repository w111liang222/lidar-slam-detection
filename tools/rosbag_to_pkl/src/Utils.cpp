#include "Utils.h"

namespace py = pybind11;

void write_pickle(std::string filename, py::dict data)
{
  py::object open   = py::module::import("builtins").attr("open");
  py::object pickle = py::module::import("pickle");
  py::object file = open(filename, "wb");
  pickle.attr("dump")(data, file, 4);
  file.attr("close")();
}

std::vector<float> matrix2vector(Eigen::Matrix4d T) {
  std::vector<float> vector(6, 0);
  Eigen::Vector3d translation(T.topRightCorner<3, 1>());
  vector[0] = translation(0);
  vector[1] = translation(1);
  vector[2] = translation(2);

  Eigen::Vector3d eulerAngle = T.topLeftCorner<3, 3>().eulerAngles (2, 0, 1);
  vector[5] = eulerAngle(0) / M_PI * 180.0;
  vector[4] = eulerAngle(1) / M_PI * 180.0;
  vector[3] = eulerAngle(2) / M_PI * 180.0;
  return vector;
}

std::vector<float> mat2vector(cv::Mat mat) {
  Eigen::Matrix4d eigen_mat;
  cv::cv2eigen(mat, eigen_mat);
  return matrix2vector(eigen_mat);
}

void write_pickle(std::string directory, int idx, py::dict data)
{
  char file_idx[256] = "";
  sprintf(file_idx, "%06d.pkl", idx);
  write_pickle(directory + "/" + file_idx, data);
}

void write_config(std::string filename, py::dict config)
{
  py::object open = py::module::import("builtins").attr("open");
  py::object yaml = py::module::import("yaml");
  py::object file = open(filename, "w");
  yaml.attr("dump")(config, file);
  file.attr("close")();
}

py::dict generate_sensor_config(cv::FileStorage &config)
{
  cv::Mat Tvl, Tvi, Tvg;
  config["extrinsic_lidar"] >> Tvl;
  config["extrinsic_imu"]   >> Tvi;
  config["extrinsic_gps"]   >> Tvg;
  cv::invert(Tvg, Tvg);

  py::dict lidar_config;
  lidar_config["name"]                 = "Custom";
  lidar_config["port"]                 = 2688;
  lidar_config["extrinsic_parameters"] = to_list(mat2vector(Tvl));
  lidar_config["range"]                = to_list(std::vector<float>{-1000.0, -1000.0, -10, 1000.0, 1000.0, 10});
  lidar_config["exclude"]              = to_list(std::vector<float>{-2.0, -2.0, -2.0, 2.0, 2.0, 4.0});

  py::dict sensor_config;
  sensor_config["lidar"]             = to_list(std::vector<py::dict>{lidar_config});
  sensor_config["camera"]            = py::list();
  sensor_config["radar"]             = py::list();
  sensor_config["ins"]               = py::dict();
  sensor_config["ins"]["ins_normal"] = py::dict();

  int gps_mode;
  config["gps_mode"] >> gps_mode;
  sensor_config["ins"]["ins_type"]                  = std::to_string(gps_mode) + "D";
  sensor_config["ins"]["extrinsic_parameters"]      = to_list(mat2vector(Tvg));
  sensor_config["ins"]["imu_extrinsic_parameters"]  = to_list(mat2vector(Tvi));
  sensor_config["ins"]["ins_normal"]["use"]         = true;
  sensor_config["ins"]["ins_normal"]["status"]      = 1;
  sensor_config["ins"]["ins_normal"]["stable_time"] = 0;
  sensor_config["ins"]["ins_normal"]["precision"]   = 1000;

  return sensor_config;
}

py::array_t<float> generate_identy_matrix()
{
  std::vector<float> f(16, 0);
  f[0]  = 1.0;
  f[5]  = 1.0;
  f[10] = 1.0;
  f[15] = 1.0;
  return py::array_t<float>(py::array::ShapeContainer({4, 4}), f.data());
}

void transformPointCloud(PointCloud::Ptr &points, Eigen::Matrix4d T)
{
  Eigen::Affine3f affine;
  affine.matrix() = T.cast<float>();
  for(size_t i = 0; i < points->size(); i++)
  {
    pcl::PointXYZ p_in(points->points[i].x, points->points[i].y, points->points[i].z);
    pcl::PointXYZ p_out = pcl::transformPoint(p_in, affine);
    points->points[i].x = p_out.x;
    points->points[i].y = p_out.y;
    points->points[i].z = p_out.z;
  }
}

py::array_t<float> scan_to_numpy_points(PointCloud::Ptr &p)
{
  int size = p->points.size();
  std::vector<float> f(size * 4);
  for (int i = 0; i < size; i++) {
    f[4 * i + 0] = p->points[i].x;
    f[4 * i + 1] = p->points[i].y;
    f[4 * i + 2] = p->points[i].z;
    f[4 * i + 3] = p->points[i].intensity / 255.0f;
  }
  return py::array_t<float>(py::array::ShapeContainer({(long) f.size() / 4, 4}), f.data());
}

py::array_t<float> scan_to_numpy_stamp (PointCloud::Ptr &p)
{
  int size = p->points.size();
  std::vector<float> f(size * 2);
  for (int i = 0; i < size; i++) {
    f[2 * i + 0] = std::max(p->points[i].time, 0.0f) * 1000.0f;
    f[2 * i + 1] = 0;
  }
  return py::array_t<float>(py::array::ShapeContainer({(long) f.size() / 2, 2}), f.data());
}

py::array_t<double> imu_to_numpy(std::vector<Imu_t> imus)
{
  int size = imus.size();
  std::vector<double> f(size * 7);
  for(int i = 0; i < size; i++) {
    f[7 * i + 0] = imus[i].timestamp;
    f[7 * i + 1] = imus[i].gyro_x / M_PI * 180.0;
    f[7 * i + 2] = imus[i].gyro_y / M_PI * 180.0;
    f[7 * i + 3] = imus[i].gyro_z / M_PI * 180.0;
    f[7 * i + 4] = imus[i].acc_x / 9.81;
    f[7 * i + 5] = imus[i].acc_y / 9.81;
    f[7 * i + 6] = imus[i].acc_z / 9.81;
  }
  return py::array_t<double>(py::array::ShapeContainer({(long) f.size() / 7, 7}), f.data());
}

py::dict ins_to_dict(Ins_t ins)
{
  py::dict ins_dict;
  ins_dict["timestamp"]     = ins.timestamp;
  ins_dict["gps_week"]      = getGPSweek(ins.timestamp);
  ins_dict["gps_time"]      = getGPSsecond(ins.timestamp);
  ins_dict["heading"]       = 0;
  ins_dict["pitch"]         = 0;
  ins_dict["roll"]          = 0;
  ins_dict["gyro_x"]        = 0;
  ins_dict["gyro_y"]        = 0;
  ins_dict["gyro_z"]        = 0;
  ins_dict["acc_x"]         = 0;
  ins_dict["acc_y"]         = 0;
  ins_dict["acc_z"]         = 0;
  ins_dict["latitude"]      = ins.latitude;
  ins_dict["longitude"]     = ins.longitude;
  ins_dict["altitude"]      = ins.altitude;
  ins_dict["Ve"]            = 0;
  ins_dict["Vn"]            = 0;
  ins_dict["Vu"]            = 0;
  ins_dict["Status"]        = ins.status;
  return ins_dict;
}

void create_directory(std::string path) {
  std::string command = "mkdir";
  command.append(" -p ");
  command.append(path);
  int ret = system(command.c_str());
  (void ) ret;
}

#define SECS_PER_WEEK (60L*60*24*7)
#define LEAP_SECOND 18
int getGPSweek(const uint64_t &stamp) {
  double diff = (stamp - 315964800000000ULL) / 1000000.0;
  return (int) (diff / SECS_PER_WEEK);
}

double getGPSsecond(const uint64_t &stamp) {
  double diff = (stamp - 315964800000000ULL) / 1000000.0;
  return (diff - (int) (diff / SECS_PER_WEEK) * SECS_PER_WEEK + LEAP_SECOND);
}