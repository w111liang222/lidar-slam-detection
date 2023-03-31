#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <thread>
#include <cmath>

#include "Logger.h"
#include "calib_wrapper.h"
#include "lidar_ins/include/sensor.h"
#include "lidar_ins/include/aligner.h"
#include "lidar_imu/include/calib_lidar_imu.h"

namespace py=pybind11;

// Lidar Ins Calibration
static std::unique_ptr<Lidar> lidar(new Lidar());
static std::unique_ptr<Odom> odom(new Odom());
static std::unique_ptr<Aligner> aligner(new Aligner(Aligner::getConfig()));
static std::unique_ptr<std::thread> optThread(nullptr);

Transform array2transform(py::array_t<float> & array) {
    auto ref = array.unchecked<2>();
    Transform::Matrix tM;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tM(i, j) = ref(i, j);
        }
    }
    return Transform(tM);
}

py::array_t<float> transform2array(Transform &T) {
    Transform::Matrix tM = T.matrix();
    std::vector<float> data(16, 0);
    for (int i = 0; i < 16; i++) {
        data[i] = tM(i / 4, i % 4);
    }
    return py::array_t<float>(py::array::ShapeContainer({4, 4}), data.data());
}

void calib_ins_reset(py::array_t<float> &initArray) {
    if (optThread != nullptr) {
        optThread->join();
        optThread.reset(nullptr);
    }
    lidar.reset(new Lidar());
    odom.reset(new Odom());
    aligner.reset(new Aligner(Aligner::getConfig()));

    Transform initTransform = array2transform(initArray);
    lidar->setOdomLidarTransform(initTransform);
    lidar->setBestOdomLidarTransform(initTransform);
    LOG_INFO("INS calibration reset success");
}

void calib_ins_set_points(py::array_t<float> &points, Timestamp time) {
    if (optThread != nullptr) {
        LOG_WARN("INS calibration optimization is running, cannot set points");
        return;
    }
    size_t totalScans = lidar->getNumberOfScans();
    LOG_INFO("INS calibration has {} scans", totalScans);
    if (totalScans >= 20) {
        return;
    }
    std::vector<float> point;
    auto ref = points.unchecked<2>();
    for (int i = 0; i < ref.shape(0); i++) {
        float x = ref(i, 0);
        float y = ref(i, 1);
        float z = ref(i, 2);
        point.emplace_back(x);
        point.emplace_back(y);
        point.emplace_back(z);
    }
    lidar->addPointcloud(point, time, Scan::getConfig());
}

void calib_ins_set_odom(py::array_t<float> &array, Timestamp time) {
    if (optThread != nullptr) {
        LOG_WARN("INS calibration optimization is running, cannot set odometry");
        return;
    }
    Transform T = array2transform(array);
    odom->addTransformData(time, T);
    LOG_INFO("INS calibration has {} odoms", odom->getNumberOfOdoms());
}

void calib_ins_calibrate_internal() {
    aligner->lidarOdomTransform(lidar.get(), odom.get());
}

void calib_ins_calibrate() {
    if (optThread != nullptr) {
        LOG_WARN("INS calibration optimization is running, waiting...");
        optThread->join();
        optThread.reset(nullptr);
    }

    lidar->setOdomOdomTransforms(*odom);
    optThread.reset(new std::thread(&calib_ins_calibrate_internal));
}

py::dict calib_ins_get_calibration() {
    float percent;
    py::dict result;
    result["finish"] = aligner->getOptimizationStatus(percent);
    result["percent"] = percent;
    return result;
}

py::array_t<float> calib_ins_get_calibration_transform() {
    Transform T = lidar->getBestOdomLidarTransform();
    return transform2array(T);
}

// Lidar Imu Calibration
static std::unique_ptr<CalibLidarImu> caliber(new CalibLidarImu());

Eigen::Matrix4d array2Eigen(py::array_t<float> &array) {
    auto ref = array.unchecked<2>();
    Eigen::Matrix4d tM;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tM(i, j) = ref(i, j);
        }
    }
    return tM;
}

void calib_imu_reset() {
    caliber.reset(new CalibLidarImu());
    LOG_INFO("IMU calibration reset success");
}

void calib_imu_set_points(py::array_t<float> &array, Timestamp stamp, py::array_t<float> &odom, py::array_t<float> &pose) {
    auto ref = array.unchecked<2>();
    size_t num = ref.shape(0);

    size_t totalScans = caliber->getNumberOfScans();
    LOG_INFO("IMU calibration has {} scans", totalScans);
    if (totalScans >= 200) {
        return;
    }

    // convert numpy to pointcloud
    CloudT::Ptr cloud(new CloudT());
    cloud->header.stamp = stamp;
    cloud->width = static_cast<int>(num);
    cloud->height = 1;
    cloud->points.resize(num);
    for (size_t i = 0; i < num; i++) {
        cloud->points[i].x         = ref(i, 0);
        cloud->points[i].y         = ref(i, 1);
        cloud->points[i].z         = ref(i, 2);
        cloud->points[i].intensity = ref(i, 3);
    }

    Eigen::Matrix4d odomT = array2Eigen(odom);
    Eigen::Matrix4d poseT = array2Eigen(pose);

    LidarData data;
    data.cloud = cloud;
    data.stamp = stamp / 1000000.0;
    caliber->addLidarData(data, poseT, odomT);
}

void calib_imu_set_imu(py::array_t<double> &array) {
    auto ref = array.unchecked<2>();
    size_t num = ref.shape(0);
    for (size_t i = 0; i < num; i++) {
        ImuData data;
        data.gyr = Eigen::Vector3d(ref(i, 1) / 180.0 * M_PI,
                                   ref(i, 2) / 180.0 * M_PI,
                                   ref(i, 3) / 180.0 * M_PI);
        data.acc = Eigen::Vector3d(ref(i, 4) * 9.81,
                                   ref(i, 5) * 9.81,
                                   ref(i, 6) * 9.81);
        data.rot = Eigen::Quaterniond::Identity();
        data.stamp = ref(i, 0) / 1000000.0;
        caliber->addImuData(data);
    }
    LOG_INFO("IMU calibration has {} imus", caliber->getNumberOfImus());
}

py::array_t<float> calib_imu_calibrate() {
    Eigen::Matrix3d result = caliber->calib(true);
    LOG_INFO("imu calibration rotation matrix:");
    std::cout << result << std::endl;

    std::vector<float> data(9, 0);
    for (int i = 0; i < 9; i++) {
        data[i] = result(i / 3, i % 3);
    }
    return py::array_t<float>(py::array::ShapeContainer({3, 3}), data.data());
}

py::list calib_imu_get_lidar_poses() {
    py::list result;
    std::vector<Eigen::Matrix4d> poses = caliber->getLidarPose();
    for(size_t i = 0; i < poses.size(); i++) {
        std::vector<float> data(16, 0);
        for (int j = 0; j < 16; j++) {
            data[j] = poses[i](j / 4, j % 4);
        }
        auto pose = py::array_t<float>(py::array::ShapeContainer({4, 4}), data.data());
        result.append(pose);
    }
    return result;
}

py::list calib_imu_get_imu_poses() {
    py::list result;
    std::vector<Eigen::Matrix4d> poses = caliber->getImuPose();
    for(size_t i = 0; i < poses.size(); i++) {
        std::vector<float> data(16, 0);
        for (int j = 0; j < 16; j++) {
            data[j] = poses[i](j / 4, j % 4);
        }
        auto pose = py::array_t<float>(py::array::ShapeContainer({4, 4}), data.data());
        result.append(pose);
    }
    return result;
}

py::array_t<float>  calib_imu_get_lidar_T_R(py::array_t<float> &array) {
    auto ref = array.unchecked<2>();
    size_t num = ref.shape(0);

    // convert numpy to pointcloud
    CloudT::Ptr cloud(new CloudT());
    cloud->width = static_cast<int>(num);
    cloud->height = 1;
    cloud->points.resize(num);
    for (size_t i = 0; i < num; i++) {
        cloud->points[i].x         = ref(i, 0);
        cloud->points[i].y         = ref(i, 1);
        cloud->points[i].z         = ref(i, 2);
        cloud->points[i].intensity = ref(i, 3);
    }

    LidarData data;
    data.cloud = cloud;
    Eigen::Matrix4d T_R =  caliber->get_lidar_T_R(data);

    std::vector<float> T(16, 0);
    for (int i = 0; i < 16; i++) {
        T[i] = T_R(i / 4, i % 4);
    }
    return py::array_t<float>(py::array::ShapeContainer({4, 4}), T.data());
}

PYBIND11_MODULE(calib_driver_ext, m) {
    m.doc() = "calibration interface";
    // Lidar Ins Calibration
    m.def("calib_ins_reset", &calib_ins_reset, "calib_ins_reset",
        py::arg("initArray"));
    m.def("calib_ins_set_points", &calib_ins_set_points, "calib_ins_set_points",
        py::arg("points"), py::arg("time"));
    m.def("calib_ins_set_odom", &calib_ins_set_odom, "calib_ins_set_odom",
        py::arg("array"), py::arg("time"));
    m.def("calib_ins_calibrate", &calib_ins_calibrate, "calib_ins_calibrate");
    m.def("calib_ins_get_calibration", &calib_ins_get_calibration, "calib_ins_get_calibration");
    m.def("calib_ins_get_calibration_transform", &calib_ins_get_calibration_transform, "calib_ins_get_calibration_transform");

    // Lidar Imu Calibration
    m.def("calib_imu_reset", &calib_imu_reset, "calib_imu_reset");
    m.def("calib_imu_set_points", &calib_imu_set_points, "calib_imu_set_points",
        py::arg("points"), py::arg("time"), py::arg("odom"), py::arg("pose"));
    m.def("calib_imu_set_imu", &calib_imu_set_imu, "calib_imu_set_imu",
        py::arg("array"));
    m.def("calib_imu_calibrate", &calib_imu_calibrate, "calib_imu_calibrate");
    m.def("calib_imu_get_lidar_poses", &calib_imu_get_lidar_poses, "calib_imu_get_lidar_poses");
    m.def("calib_imu_get_imu_poses", &calib_imu_get_imu_poses, "calib_imu_get_imu_poses");
    m.def("calib_imu_get_lidar_T_R", &calib_imu_get_lidar_T_R, "calib_imu_get_lidar_T_R");
}
