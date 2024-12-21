#include "cpp_utils.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <sys/prctl.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#ifdef HAVE_CUDA_ENABLE
#include "cuda_runtime.h"
#endif

#include "Utils.h"
#include "Transform.h"
#include "SystemUtils.h"
#include "InterProcess.h"
#include "UTMProjector.h"
#include "KalmanFilter.h"

namespace py=pybind11;

void init_backtrace_handle(void) {
    backtrace_handle_init();
}

void set_thread_priority(std::string name, int priority) {
    prctl(PR_SET_NAME, name.c_str(), 0, 0, 0);
    setSelfThreadPriority(priority);
}

template <typename T>
py::bytes message_to_bytes(T &msg) {
    std::string encode_str(msg.getEncodedSize(), 0);
    msg.encode(encode_str.data(), 0, encode_str.size());
    return py::bytes(encode_str);
}

py::bytes publish_message(std::string filename) {
    if (ends_with(filename, ".pcd")) {
        pcl::PCLPointCloud2 cloud;
        pcl::PCDReader reader;
        reader.read(filename, cloud);

        // to pointcloud message
        sensor_msgs::PointCloud msg;
        fromPCL(cloud, msg, true);
        return message_to_bytes(msg);
    }

    return py::bytes();
}

void set_logger_level(std::string level) {
    set_perception_log_level(level);
}

void set_message_core(bool enable) {
    set_core_enable(enable);
    LOG_INFO("message system is set to enable");
}

py::array_t<double> get_projection_forward(double lat0, double lon0, double lat1, double lon1) {
    UTMProjector projector;
    std::vector<double> data(2, 0);
    double originX, originY;
    double dataX, dataY;
    projector.FromGlobalToLocal(lat0, lon0, originX, originY);
    projector.FromGlobalToLocal(lat1, lon1, dataX, dataY);
    data[0] = dataX - originX;
    data[1] = dataY - originY;
    return py::array_t<double>(py::array::ShapeContainer({2}), data.data());
}

py::array_t<double> get_projection_backward(double lat0, double lon0, double x, double y) {
    UTMProjector projector;
    std::vector<double> data(2, 0);
    double originX, originY;
    projector.FromGlobalToLocal(lat0, lon0, originX, originY);
    x = x + originX;
    y = y + originY;
    projector.FromLocalToGlobal(x, y, data[0], data[1]);
    return py::array_t<double>(py::array::ShapeContainer({2}), data.data());
}

py::array_t<float> get_transform_from_RPYT(double x, double y, double z,
                                           double yaw, double pitch, double roll) {
    Transform trans = getTransformFromRPYT(x, y, z, yaw, pitch, roll);
    Matrix tM = trans.matrix();
    std::vector<float> data(16, 0);
    for (int i = 0; i < 16; i++) {
        data[i] = tM(i / 4, i % 4);
    }
    return py::array_t<float>(py::array::ShapeContainer({4, 4}), data.data());
}

py::array_t<float> get_RPYT_from_transform(const py::array_t<float> &transform) {
    auto ref_transform = transform.unchecked<2>();
    Matrix tM;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tM(i, j) = ref_transform(i, j);
        }
    }
    Transform trans = Transform(tM);
    double x, y, z, yaw, pitch, roll;
    getRPYTfromTransformFrom(trans, x, y, z, yaw, pitch, roll);
    std::vector<float> data(6, 0);
    data[0] = x;
    data[1] = y;
    data[2] = z;
    data[3] = yaw;
    data[4] = pitch;
    data[5] = roll;
    return py::array_t<float>(py::array::ShapeContainer({6}), data.data());
}

Transform getGlobalTransform(UTMProjector &projector,
                             double lat, double lon, double alt,
                             double yaw, double pitch, double roll) {
    double dataX, dataY;
    projector.FromGlobalToLocal(lat, lon, dataX, dataY);
    yaw = yaw - get_grid_convergence(projector.GetLongitude0(), lat, lon);
    return getTransformFromRPYT(dataX, dataY, alt, -yaw, pitch, roll);
}

py::array_t<double> getRelativeTransform(double lat0, double lon0, double alt0,
                                         double yaw0, double pitch0, double roll0,
                                         double lat1, double lon1, double alt1,
                                         double yaw1, double pitch1, double roll1,
                                         double x, double y, double z, double h, double p, double r) {
    UTMProjector projector;
    Transform T0 = getGlobalTransform(projector, lat0, lon0, alt0, yaw0, pitch0, roll0);
    Transform T1 = getGlobalTransform(projector, lat1, lon1, alt1, yaw1, pitch1, roll1);
    Transform staticTransform = getTransformFromRPYT(x, y, z, h, p, r);
    Transform dt = staticTransform.inverse() * (T1.inverse() * T0) * staticTransform;

    Matrix tM = dt.matrix();
    std::vector<double> data(16, 0);
    for (int i = 0; i < 16; i++) {
        data[i] = tM(i / 4, i % 4);
    }
    return py::array_t<double>(py::array::ShapeContainer({4, 4}), data.data());
}

py::array_t<double> computeRTKTransform(double lat0, double lon0, double alt0,
                                        double yaw0, double pitch0, double roll0,
                                        double lat1, double lon1, double alt1,
                                        double yaw1, double pitch1, double roll1) {
    UTMProjector projector;
    double originX, originY;
    double dataX, dataY, dataZ;
    projector.FromGlobalToLocal(lat0, lon0, originX, originY);
    projector.FromGlobalToLocal(lat1, lon1, dataX, dataY);
    dataX = dataX - originX;
    dataY = dataY - originY;
    dataZ = alt1 - alt0;

    yaw1 = yaw1 - get_grid_convergence(projector.GetLongitude0(), lat1, lon1);
    double dataYaw = -yaw1;
    double dataPitch = pitch1;
    double dataRoll = roll1;
    Transform dt = getTransformFromRPYT(dataX, dataY, dataZ, dataYaw, dataPitch, dataRoll);

    Matrix tM = dt.matrix();
    std::vector<double> data(16, 0);
    for (int i = 0; i < 16; i++) {
        data[i] = tM(i / 4, i % 4);
    }
    return py::array_t<double>(py::array::ShapeContainer({4, 4}), data.data());
}

static pcl::Filter<pcl::PointXYZI>::Ptr sDownsampler;
static float sVoxelSize = -1.0;
py::array_t<float> pointcloud_downsample_pcl(py::array_t<float> &points, float voxel_size) {
    auto ref = points.unchecked<2>();
    size_t num = ref.shape(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    points_ptr->width = static_cast<int>(num);
    points_ptr->height = 1;
    points_ptr->points.resize(num);
    for (size_t i = 0; i < num; i++) {
        points_ptr->points[i].x         = ref(i, 0);
        points_ptr->points[i].y         = ref(i, 1);
        points_ptr->points[i].z         = ref(i, 2);
        points_ptr->points[i].intensity = ref(i, 3);
    }

    if (fabs(sVoxelSize - voxel_size) > 1e-3) {
        sVoxelSize = voxel_size;
        auto voxelgrid = new pcl::VoxelGrid<pcl::PointXYZI>();
        voxelgrid->setLeafSize(voxel_size, voxel_size, voxel_size);
        sDownsampler.reset(voxelgrid);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr frame_downsample(new pcl::PointCloud<pcl::PointXYZI>());
    sDownsampler->setInputCloud(points_ptr);
    sDownsampler->filter(*frame_downsample);

    std::vector<float> f;
    for (auto p : frame_downsample->points) {
        f.emplace_back(p.x);
        f.emplace_back(p.y);
        f.emplace_back(p.z);
        f.emplace_back(p.intensity);
    }
    return py::array_t<float>(py::array::ShapeContainer({(long) f.size() / 4, 4}), f.data());
}

py::array_t<float> pointcloud_downsample(py::array_t<float> &points, float voxel_size) {
    return pointcloud_downsample_pcl(points, voxel_size);
}

std::vector<py::array_t<int>> get_association(int det_len, int trk_len,
                                              const py::array_t<int> &matched_indices,
                                              float threshold,
                                              const py::array_t<float> &iou_matrix) {
    std::set<int> det_set;
    std::set<int> trk_set;
    auto ref_match = matched_indices.unchecked<2>();
    int match_len = ref_match.shape(0);
    for(int i = 0; i < match_len; i++) {
        det_set.insert(ref_match(i, 0));
        trk_set.insert(ref_match(i, 1));
    }

    std::vector<int> unmatched_detections;
    std::vector<int> unmatched_trackers;
    for(int i = 0; i < det_len; i++) {
        if (det_set.find(i) == det_set.end()) {
            unmatched_detections.emplace_back(i);
        }
    }
    for(int i = 0; i < trk_len; i++) {
        if (trk_set.find(i) == trk_set.end()) {
            unmatched_trackers.emplace_back(i);
        }
    }

    auto ref_iou = iou_matrix.unchecked<2>();
    std::vector<int> match;
    for(int i = 0; i < match_len; i++) {
        int det_id = ref_match(i, 0);
        int trk_id = ref_match(i, 1);
        if (ref_iou(det_id, trk_id) > threshold) {
            unmatched_detections.emplace_back(det_id);
            unmatched_trackers.emplace_back(trk_id);
        } else {
            match.emplace_back(det_id);
            match.emplace_back(trk_id);
        }
    }

    auto matched_n = py::array_t<int>(py::array::ShapeContainer({match.size() / 2, 2}), match.data());
    auto unmatched_detections_n = py::array_t<int>(py::array::ShapeContainer({unmatched_detections.size()}), unmatched_detections.data());
    auto unmatched_trackers_n = py::array_t<int>(py::array::ShapeContainer({unmatched_trackers.size()}), unmatched_trackers.data());
    return std::vector<py::array_t<int>>({matched_n, unmatched_detections_n, unmatched_trackers_n});
}

static std::map<uint64_t, std::shared_ptr<KalmanFilter>> filters;
void init_filters() {
    for(int i = 0; i < 65535; i++) {
        filters[i] = std::make_shared<KalmanFilter>();
    }
}
void use_filter(int handle, bool is_static, const py::array_t<float> &x) {
    if (filters.find(handle) == filters.end()) {
        LOG_WARN("Kalman filter ({}) is not found", handle);
        return;
    }
    std::shared_ptr<KalmanFilter> filter = filters[handle];

    auto ref_x = x.unchecked<1>();
    auto dim = ref_x.shape(0);
    Eigen::VectorXd x_vec = Eigen::VectorXd(dim);
    for(int i = 0; i < dim; i++) {
        x_vec(i) = ref_x(i);
    }
    filter->Initialization(is_static, x_vec);
}

py::array_t<float> filter_predict(int handle, double dt, py::array_t<float> &motion, float dh) {
    if (filters.find(handle) == filters.end()) {
        LOG_WARN("Kalman filter ({}) is not found", handle);
        return py::array_t<float>(py::array::ShapeContainer({0}));
    }
    std::shared_ptr<KalmanFilter> filter = filters[handle];

    auto ref = motion.unchecked<2>();
    Eigen::Matrix<float, 4, 4> T;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T(i, j) = ref(i, j);
        }
    }

    filter->Prediction(dt, T, dh);
    Eigen::VectorXd &vec_x = filter->GetX();
    auto dim = vec_x.size();
    std::vector<float> x(dim);
    for(int i = 0; i < dim; i++) {
        x[i] = vec_x(i);
    }
    return py::array_t<float>(py::array::ShapeContainer({dim}), x.data());
}

py::array_t<float> filter_update(int handle, py::array_t<float> &z) {
    if (filters.find(handle) == filters.end()) {
        LOG_WARN("Kalman filter ({}) is not found", handle);
        return py::array_t<float>(py::array::ShapeContainer({0}));
    }
    std::shared_ptr<KalmanFilter> filter = filters[handle];

    auto ref_z = z.unchecked<1>();
    auto dim_z = ref_z.shape(0);
    Eigen::VectorXd z_vec = Eigen::VectorXd(dim_z);
    for(int i = 0; i < dim_z; i++) {
        z_vec(i) = ref_z(i);
    }
    filter->KFUpdate(z_vec);

    Eigen::VectorXd &vec_x = filter->GetX();
    auto dim = vec_x.size();
    std::vector<float> x(dim);
    for(int i = 0; i < dim; i++) {
        x[i] = vec_x(i);
    }
    return py::array_t<float>(py::array::ShapeContainer({dim}), x.data());
}

void motion_prediction(int handle, py::array_t<float> &trajectory) {
    if (filters.find(handle) == filters.end()) {
        LOG_WARN("filter ({}) is not found", handle);
        return;
    }
    std::shared_ptr<KalmanFilter> filter = filters[handle];

    auto traj_rw = trajectory.mutable_unchecked<3>();
    Eigen::MatrixXd F = filter->F_;
    Eigen::VectorXd x = filter->x_;
    F(0, 7)  = 0.1;
    F(1, 8)  = 0.1;
    F(6, 9)  = 0.1;
    F(7, 10) = 0.1;
    x(10) = 0;
    auto dim = traj_rw.shape(1);
    for(int i = 0; i < dim; i++) {
        x = F * x;
        x(9) = x(9) * 0.95;
        float vx = x(7) * cos(x(9) * 0.1) - x(8) * sin(x(9) * 0.1);
        float vy = x(7) * sin(x(9) * 0.1) + x(8) * cos(x(9) * 0.1);
        x(7) = vx;
        x(8) = vy;
        traj_rw(0, i, 0) = x(0);
        traj_rw(0, i, 1) = x(1);
        traj_rw(0, i, 2) = x(2);
        traj_rw(0, i, 3) = x(6);
        traj_rw(0, i, 4) = x(7);
        traj_rw(0, i, 5) = x(8);
        traj_rw(0, i, 6) = (i + 1) * 100000;
    }
}

PYBIND11_MODULE(cpp_utils_ext, m) {
    m.doc() = "cpp uilts python interface";

    m.def("init_backtrace_handle", &init_backtrace_handle, "init_backtrace_handle");

    m.def("set_thread_priority", &set_thread_priority, "set_thread_priority",
          py::arg("name"), py::arg("priority")
    );

    m.def("set_logger_level", &set_logger_level, "set_logger_level",
          py::arg("level")
    );

    m.def("set_message_core", &set_message_core, "set_message_core",
          py::arg("enable")
    );

    m.def("publish_message", &publish_message, "publish_message",
          py::arg("filename")
    );

    m.def("get_projection_forward", &get_projection_forward, "get_projection_forward",
          py::arg("lat0"), py::arg("lon0"),
          py::arg("lat1"), py::arg("lon1")
    );

    m.def("get_projection_backward", &get_projection_backward, "get_projection_backward",
          py::arg("lat0"), py::arg("lon0"),
          py::arg("x"), py::arg("y")
    );

    m.def("get_transform_from_RPYT", &get_transform_from_RPYT, "get_transform_from_RPYT",
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("yaw"), py::arg("pitch"), py::arg("roll")
    );

    m.def("get_RPYT_from_transform", &get_RPYT_from_transform, "get_RPYT_from_transform",
          py::arg("transform")
    );

    m.def("getRelativeTransform", &getRelativeTransform, "getRelativeTransform",
          py::arg("lat0"), py::arg("lon0"), py::arg("alt0"),
          py::arg("yaw0"), py::arg("pitch0"), py::arg("roll0"),
          py::arg("lat1"), py::arg("lon1"), py::arg("alt1"),
          py::arg("yaw1"), py::arg("pitch1"), py::arg("roll1"),
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("h"), py::arg("p"), py::arg("r")
    );

    m.def("computeRTKTransform", &computeRTKTransform, "computeRTKTransform",
          py::arg("lat0"), py::arg("lon0"), py::arg("alt0"),
          py::arg("yaw0"), py::arg("pitch0"), py::arg("roll0"),
          py::arg("lat1"), py::arg("lon1"), py::arg("alt1"),
          py::arg("yaw1"), py::arg("pitch1"), py::arg("roll1")
    );

    m.def("pointcloud_downsample", &pointcloud_downsample, "pointcloud_downsample",
          py::arg("points"), py::arg("voxel_size")
    );

    m.def("get_association", &get_association, "get_association",
          py::arg("det_len"), py::arg("trk_len"), py::arg("matched_indices"),
          py::arg("threshold"), py::arg("iou_matrix")
    );

    m.def("init_filters", &init_filters, "init_filters");

    m.def("use_filter", &use_filter, "use_filter",
          py::arg("handle"), py::arg("is_static"), py::arg("x")
    );

    m.def("filter_predict", &filter_predict, "filter_predict",
          py::arg("handle"), py::arg("dt"), py::arg("motion"), py::arg("dh")
    );

    m.def("filter_update", &filter_update, "filter_update",
          py::arg("handle"), py::arg("z")
    );

    m.def("motion_prediction", &motion_prediction, "motion_prediction",
          py::arg("handle"), py::arg("trajectory")
    );

}