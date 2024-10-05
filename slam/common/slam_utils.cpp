#include "slam_utils.h"
#include <cmath>
#include <dirent.h>
#include <unistd.h>
#include <sys/time.h>
#include <omp.h>

#include <pcl/common/transforms.h>

int hex2dec(char ch) {
  if ('0' <= ch && ch <= '9') return ch - '0';
  if ('A' <= ch && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

char dec2hex(int d) {
  if (0 <= d && d <=9) return d + '0';
  if (d >= 10) return d - 10 + 'A';
  return '\0';
}

uint64_t hashCoordinate(const double &px, const double &py, const double &pz) {
    uint64_t x = int(px * 10.0) + 1e5; // 0 ~ 200000
    uint64_t y = int(py * 10.0) + 1e5; // 0 ~ 200000
    uint64_t z = int(pz * 10.0) + 1e3; // 0 ~ 2000
    uint64_t hash = (x << 30) | (y << 12) | (z);
    return hash;
}

std::vector<std::string> getDirFiles(std::string path, std::string extension) {
    std::vector<std::string> files;
    DIR *dir = opendir(path.c_str());
    if (dir == NULL) {
        LOG_WARN("{} open directory error", path);
        return files;
    }

    struct dirent *d_ent = NULL;
    while ((d_ent = readdir(dir)) != NULL) {
        if (d_ent->d_type != DT_DIR) {
            std::string d_name(d_ent->d_name);
            if (strcmp(d_name.c_str() + d_name.length() - extension.length(), extension.c_str()) == 0) {
                std::string file = std::string(path + "/" + d_ent->d_name);
                files.push_back(file);
            }
        }
    }
    closedir(dir);
    return files;
}

std::vector<std::string> getDirDirs(std::string path) {
    std::vector<std::string> dirs;
    DIR *dir = opendir(path.c_str());
    if (dir == NULL) {
        LOG_WARN("{} open directory error", path);
        return dirs;
    }

    struct dirent *d_ent = NULL;
    while ((d_ent = readdir(dir)) != NULL) {
        int type = d_ent->d_type;
        if (type == DT_UNKNOWN) {
            std::string sub_path = std::string(path + "/" + d_ent->d_name);
            DIR* sub_dir = opendir(sub_path.c_str());
            if (sub_dir != NULL) {
                closedir(sub_dir);
                type = DT_DIR;
            }
        }
        if (type == DT_DIR) {
            std::string d_name(d_ent->d_name);
            if (d_name.compare(".") == 0 || d_name.compare("..") == 0) {
                continue;
            }
            std::string file = std::string(path + "/" + d_ent->d_name);
            dirs.push_back(file);
        }
    }
    closedir(dir);
    return dirs;
}

bool isFileExist(const std::string& name) {
    return (access(name.c_str(), F_OK) != -1);
}

const double Ang2Rad = 0.01745329251994;
Eigen::Matrix4d getTransformFromRPYT(double x, double y, double z,
                                     double yaw, double pitch, double roll) {
    Eigen::AngleAxisd rollAngle(roll * Ang2Rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch * Ang2Rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(yaw * Ang2Rad, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d trans(x, y, z);
    return (trans * yawAngle * pitchAngle * rollAngle).matrix();
}

void getRPYTfromTransformFrom(Eigen::Matrix4d tran, double &x, double &y, double &z,
                              double &yaw, double &pitch, double &roll) {
  Eigen::Vector3d translation(tran.topRightCorner<3, 1>());
  x = translation(0);
  y = translation(1);
  z = translation(2);

  Eigen::Vector3d eulerAngle =
      tran.topLeftCorner<3, 3>().eulerAngles(2, 0, 1);
  yaw = eulerAngle(0) / Ang2Rad;
  pitch = eulerAngle(1) / Ang2Rad;
  roll = eulerAngle(2) / Ang2Rad;
}

Eigen::Matrix4d computeRTKTransform(UTMProjector &projector, std::shared_ptr<RTKType> &data, Eigen::Vector3d &zero_utm) {
    double dataX, dataY, dataZ;
    projector.FromGlobalToLocal(data->latitude, data->longitude, dataX, dataY);
    dataX -= zero_utm(0);
    dataY -= zero_utm(1);
    dataZ = data->altitude - zero_utm(2);

    double heading = data->heading - get_grid_convergence(projector.GetLongitude0(), data->latitude, data->longitude);
    double dataYaw = -heading;
    double dataPitch = data->pitch;
    double dataRoll = data->roll;
    return getTransformFromRPYT(dataX, dataY, dataZ, dataYaw, dataPitch, dataRoll);
}

Eigen::Matrix4d interpolateTransform(Eigen::Matrix4d &preT, Eigen::Matrix4d &nextT, double &t_diff_ratio) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    Eigen::Matrix3d rotation_matrix = preT.block<3, 3>(0, 0);
    Eigen::Quaterniond rotation(rotation_matrix);
    Eigen::Quaterniond rotation_inverted(rotation.w(), -rotation.x(), -rotation.y(), -rotation.z());
    Eigen::Vector3d translation = preT.block<3, 1>(0, 3);
    Eigen::Vector3d trans_inverted = -(rotation_inverted * translation);

    Eigen::Quaterniond next_rotation(nextT.block<3, 3>(0, 0));
    Eigen::Vector3d next_translation = nextT.block<3, 1>(0, 3);

    Eigen::Vector3d delta_translation = trans_inverted + rotation_inverted * next_translation;
    Eigen::Quaterniond delta_rotation = rotation_inverted * next_rotation;
    Eigen::AngleAxisd angle_axis(delta_rotation);
    Eigen::Matrix<double, 6, 1> log_vector = t_diff_ratio * (Eigen::Matrix<double, 6, 1>() << delta_translation, angle_axis.angle() * angle_axis.axis()).finished();

    constexpr double kEpsilon = 1e-8;
    const double norm = log_vector.tail<3>().norm();
    if (norm < kEpsilon) {
      Eigen::Vector3d tmp_translation = log_vector.head<3>();
      Eigen::Quaterniond tmp_rotation(Eigen::Quaterniond::Identity());
      Eigen::Vector3d new_translation = translation + rotation * tmp_translation;
      Eigen::Quaterniond new_rotation = rotation * tmp_rotation;
      result.block<3, 1>(0, 3) = new_translation;
      result.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
    } else {
      Eigen::Vector3d tmp_translation = log_vector.head<3>();
      Eigen::Quaterniond tmp_rotation(Eigen::AngleAxisd(norm,  log_vector.tail<3>() / norm));
      Eigen::Vector3d new_translation = translation + rotation * tmp_translation;
      Eigen::Quaterniond new_rotation = rotation * tmp_rotation;
      result.block<3, 1>(0, 3) = new_translation;
      result.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
    }
    return result;
}

void undistortPoints(const Eigen::Matrix4f &delta_pose, PointCloudAttrPtr &points, double scan_period) {
    Eigen::Vector3f delta_translation = delta_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf delta_rotation(delta_pose.block<3, 3>(0, 0));
    Eigen::AngleAxisf angle_axis(delta_rotation);

    int num = points->cloud->points.size();
    omp_set_num_threads(4);
    #pragma omp parallel for
    for(int i = 0; i < num; i++) {
        float t_diff_ratio = (points->attr[i].stamp / 1000000.0) / scan_period;
        Eigen::Matrix<float, 6, 1> log_vector = t_diff_ratio * (Eigen::Matrix<float, 6, 1>() << delta_translation, angle_axis.angle() * angle_axis.axis()).finished();

        Eigen::Affine3f pcl_transform = Eigen::Affine3f::Identity();
        constexpr double kEpsilon = 1e-8;
        const float norm = log_vector.tail<3>().norm();
        if (norm < kEpsilon) {
            Eigen::Vector3f new_translation = log_vector.head<3>();
            Eigen::Quaternionf new_rotation(Eigen::Quaternionf::Identity());
            pcl_transform.matrix().block<3, 1>(0, 3) = new_translation;
            pcl_transform.matrix().block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
        } else {
            Eigen::Vector3f new_translation = log_vector.head<3>();
            Eigen::Quaternionf new_rotation(Eigen::AngleAxisf(norm,  log_vector.tail<3>() / norm));
            pcl_transform.matrix().block<3, 1>(0, 3) = new_translation;
            pcl_transform.matrix().block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
        }
        points->cloud->points[i] = pcl::transformPoint(points->cloud->points[i], pcl_transform);
    }
}

void undistortPoints(std::vector<PoseType> &poses, PointCloudAttrPtr &points) {
    int idx = 0;
    int num = points->cloud->points.size();
    for (int i = 1; i < poses.size(); i++) {
        double scan_period    = (poses[i].timestamp - poses[0].timestamp) / 1000000.0;

        Eigen::Matrix4f delta_matrix = poses[i].T.cast<float>();
        Eigen::Vector3f delta_translation = delta_matrix.block<3, 1>(0, 3);
        Eigen::Quaternionf delta_rotation(delta_matrix.block<3, 3>(0, 0));
        Eigen::AngleAxisf angle_axis(delta_rotation);
        for(; idx < num; idx++) {
            if (points->attr[idx].stamp > (poses[i].timestamp - points->cloud->header.stamp)) {
                break;
            }
            float t_diff_ratio = (points->attr[idx].stamp / 1000000.0) / scan_period;
            Eigen::Matrix<float, 6, 1> log_vector = t_diff_ratio * (Eigen::Matrix<float, 6, 1>() << delta_translation, angle_axis.angle() * angle_axis.axis()).finished();

            Eigen::Affine3f pcl_transform = Eigen::Affine3f::Identity();
            constexpr double kEpsilon = 1e-8;
            const float norm = log_vector.tail<3>().norm();
            if (norm < kEpsilon) {
                Eigen::Vector3f new_translation = log_vector.head<3>();
                Eigen::Quaternionf new_rotation(Eigen::Quaternionf::Identity());
                pcl_transform.matrix().block<3, 1>(0, 3) = new_translation;
                pcl_transform.matrix().block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
            } else {
                Eigen::Vector3f new_translation = log_vector.head<3>();
                Eigen::Quaternionf new_rotation(Eigen::AngleAxisf(norm,  log_vector.tail<3>() / norm));
                pcl_transform.matrix().block<3, 1>(0, 3) = new_translation;
                pcl_transform.matrix().block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
            }
            points->cloud->points[idx] = pcl::transformPoint(points->cloud->points[idx], pcl_transform);
        }
    }
}

void imageCvtColor(cv::Mat &image) {
    // convert colorspace from YUV I420 to BGR
    if (image.type() == CV_8UC1) {
      cv::cvtColor(image, image, cv::COLOR_YUV2BGR_I420);
    }
}

void pointsDistanceFilter(const PointCloud::ConstPtr& cloud, PointCloud::Ptr& filtered, double min_range, double max_range) {
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const Point& p) {
        float abs_x = std::fabs(p.x);
        float abs_y = std::fabs(p.y);
        return (abs_x >  min_range && abs_x <  max_range && abs_y >  min_range && abs_y <  max_range);
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    filtered->header = cloud->header;
}

PointCloudAttrPtr mergePoints(const std::string &base, std::map<std::string, PointCloudAttrPtr> &points, const uint64_t &scan_period) {
    if (points.find(base) == points.end()) {
        LOG_WARN("can not find {} to merge points", base);
        return PointCloudAttrPtr(new PointCloudAttr());
    }

    PointCloudAttrPtr pointcloud = points[base];
    for (auto &cloud : points) {
        if (cloud.first.compare(base) == 0) {
            continue;
        }

        double time_shift = double(cloud.second->cloud->header.stamp) - double(pointcloud->cloud->header.stamp);
        for (size_t i = 0; i < cloud.second->attr.size(); i++) {
            double stamp = double(cloud.second->attr[i].stamp) + time_shift;
            if (stamp >= 0 && stamp <= scan_period) {
                cloud.second->attr[i].stamp = uint32_t(stamp);
                pointcloud->cloud->push_back(cloud.second->cloud->points[i]);
                pointcloud->attr.push_back(cloud.second->attr[i]);
            }
        }
    }

    return pointcloud;
}

bool parseGPCHC(std::string message, RTKType &ins) {
    std::size_t head = message.find("$GPCHC");
    std::size_t tail = message.find("*");
    if (head == std::string::npos || tail == std::string::npos) {
        return false;
    }

    // keep the last 2 bytes as the checksum
    if ((message.size() - tail) < 3) {
        return false;
    }
    std::string msg = message.substr(head, tail - head + 3);
    message.erase(0, tail + 3);

    char Cs[4] = "";
    int gps_week;
    double gps_time;
    double baseline;
    int NSV1, NSV2, age, Warnning;
    int result = sscanf(msg.c_str(), "$GPCHC,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d%s",
            &gps_week, &gps_time, &ins.heading, &ins.pitch, &ins.roll, &ins.gyro_x, &ins.gyro_y, &ins.gyro_z,
            &ins.acc_x, &ins.acc_y, &ins.acc_z, &ins.latitude, &ins.longitude, &ins.altitude, &ins.Ve, &ins.Vn, &ins.Vu,
            &baseline, &NSV1, &NSV2, &ins.status, &age, &Warnning, Cs);
    bool found = (result == 24);

    // check the checksum
    if (found) {
        uint8_t checksum = 0;
        if (Cs[0] == ',') {
            // CGI-610
            checksum = hex2dec(Cs[2]) * 16 + hex2dec(Cs[3]);
        } else if (Cs[0] == '*') {
            // P2
            checksum = hex2dec(Cs[1]) * 16 + hex2dec(Cs[2]);
        } else {
            LOG_WARN("Unknown INS checksum {}", Cs);
            found = false;
        }

        uint8_t datasum = 0;
        for (int i = 1; i < msg.size() - 3; i++) {
            datasum ^= msg[i];
        }
        if (checksum != datasum) {
            LOG_WARN("INS checksum is not match {}, {}", checksum, datasum);
            found = false;
        }
    }

    if (found) {
        ins.timestamp = gps2Utc(gps_week, gps_time);
        // printf("gps_week: %d\n", ins.gps_week);
        // printf("gps_time: %lf\n", ins.gps_time);
        // printf("heading: %lf\n", ins.heading);
        // printf("latitude: %lf\n", ins.latitude);
        // printf("longitude: %lf\n", ins.longitude);
    }
    return found;
}

std::string formatGPCHC(RTKType &ins) {
    char str[1024] = "";
    int gps_week = getGPSweek(ins.timestamp);
    double gps_time = getGPSsecond(ins.timestamp);
    sprintf(str, "$GPCHC,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%.10lf,%.10lf,%.10lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,",
            gps_week, gps_time, ins.heading, ins.pitch, ins.roll, ins.gyro_x, ins.gyro_y, ins.gyro_z,
            ins.acc_x, ins.acc_y, ins.acc_z, ins.latitude, ins.longitude, ins.altitude, ins.Ve, ins.Vn, ins.Vu,
            0.0, 0, 0, ins.status, 0, 0);

    std::string msg = std::string(str);
    uint8_t datasum = 0;
    for (int i = 1; i < msg.size(); i++) {
        datasum ^= msg[i];
    }
    char Cs[4] = "";
    Cs[0] = '*';
    Cs[1] = dec2hex(datasum / 16);
    Cs[2] = dec2hex(datasum % 16);
    Cs[3] = '\0';

    msg = msg + std::string(Cs);
    return msg;
}