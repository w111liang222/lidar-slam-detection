#include "colmap_utils.h"

#include <iostream>
#include <fstream>

#include "SystemUtils.h"

template <typename T>
T ReverseBytes(const T& data) {
    T data_reversed = data;
    std::reverse(reinterpret_cast<char*>(&data_reversed),
                 reinterpret_cast<char*>(&data_reversed) + sizeof(T));
    return data_reversed;
}

inline bool IsLittleEndian() {
#ifdef BOOST_BIG_ENDIAN
    return false;
#else
    return true;
#endif
}

template <typename T>
T LittleEndianToNative(const T x) {
    if (IsLittleEndian()) {
        return x;
    } else {
        return ReverseBytes(x);
    }
}

template <typename T>
T NativeToLittleEndian(const T x) {
    if (IsLittleEndian()) {
        return x;
    } else {
        return ReverseBytes(x);
    }
}

template <typename T>
void WriteBinaryLittleEndian(std::ostream* stream, const T& data) {
    const T data_little_endian = NativeToLittleEndian(data);
    stream->write(reinterpret_cast<const char*>(&data_little_endian), sizeof(T));
}

template <typename T>
void WriteBinaryLittleEndian(std::ostream* stream, const std::vector<T>& data) {
    for (const auto& elem : data) {
        WriteBinaryLittleEndian<T>(stream, elem);
    }
}


void WriteCamerasBinary(const std::string& path, std::unordered_map<std::string, ColmapCamera>& cameras) {
    std::ofstream file(path + "cameras.bin", std::ios::trunc | std::ios::binary);
    if (!file.is_open()) {
        return;
    }

    WriteBinaryLittleEndian<uint64_t>(&file, cameras.size());
    for (const auto& camera : cameras) {
        WriteBinaryLittleEndian<uint32_t>(&file, camera.second.camera_id);
        WriteBinaryLittleEndian<int>(&file, static_cast<int>(camera.second.model_id));
        WriteBinaryLittleEndian<uint64_t>(&file, camera.second.width);
        WriteBinaryLittleEndian<uint64_t>(&file, camera.second.height);
        for (const double& param : camera.second.params) {
            WriteBinaryLittleEndian<double>(&file, param);
        }
    }
}

void WriteImagesBinary(const std::string& path, std::vector<ColmapImage> &images) {
    std::ofstream file(path + "images.bin", std::ios::trunc | std::ios::binary);
    if (!file.is_open()) {
        return;
    }

    WriteBinaryLittleEndian<uint64_t>(&file, images.size());
    for (const auto& image : images) {
        WriteBinaryLittleEndian<uint32_t>(&file, image.image_id);
        WriteBinaryLittleEndian<double>(&file, image.quat.w());
        WriteBinaryLittleEndian<double>(&file, image.quat.x());
        WriteBinaryLittleEndian<double>(&file, image.quat.y());
        WriteBinaryLittleEndian<double>(&file, image.quat.z());
        WriteBinaryLittleEndian<double>(&file, image.trans.x());
        WriteBinaryLittleEndian<double>(&file, image.trans.y());
        WriteBinaryLittleEndian<double>(&file, image.trans.z());
        WriteBinaryLittleEndian<uint32_t>(&file, image.camera_id);

        const std::string name = image.image_name + '\0';
        file.write(name.c_str(), name.size());

        WriteBinaryLittleEndian<uint64_t>(&file, image.points2d.size());
        for (const auto& point_2d : image.points2d) {
            WriteBinaryLittleEndian<double>(&file, point_2d.x);
            WriteBinaryLittleEndian<double>(&file, point_2d.y);
            WriteBinaryLittleEndian<uint64_t>(&file, point_2d.point3d_id);
        }
    }
}

void WriteImages(const std::string& path, std::vector<ColmapImage> &images) {
    for (auto& image : images) {
        if (!image.image.empty()) {
            std::string file_name = path + "images/" + image.image_name;
            cv::imwrite(file_name, image.image);
            image.image = cv::Mat();
        }
    }
}

void WritePoints3DBinary(const std::string& path, std::vector<ColmapPoint3d>& points3d) {
    std::ofstream file(path + "points3D.bin", std::ios::trunc | std::ios::binary);
    if (!file.is_open()) {
        return;
    }

    WriteBinaryLittleEndian<uint64_t>(&file, points3d.size());
    for (const auto &point : points3d) {
        WriteBinaryLittleEndian<uint64_t>(&file, point.point_id);
        WriteBinaryLittleEndian<double>(&file, point.pose.x());
        WriteBinaryLittleEndian<double>(&file, point.pose.y());
        WriteBinaryLittleEndian<double>(&file, point.pose.z());
        WriteBinaryLittleEndian<uint8_t>(&file, static_cast<uint8_t>(point.color.x()));
        WriteBinaryLittleEndian<uint8_t>(&file, static_cast<uint8_t>(point.color.y()));
        WriteBinaryLittleEndian<uint8_t>(&file, static_cast<uint8_t>(point.color.z()));
        WriteBinaryLittleEndian<double>(&file, 0.0);
        WriteBinaryLittleEndian<uint64_t>(&file, point.tracks.size());
        for (const auto& track : point.tracks) {
            WriteBinaryLittleEndian<uint32_t>(&file, track.image_id);
            WriteBinaryLittleEndian<uint32_t>(&file, track.point2d_id);
        }
    }
}

ColmapUtil::ColmapUtil(std::string path) {
    mCameraIdx = 1;
    mImageIdx  = 1;
    mPointIdx  = 1;
    mRGBPoints = PointCloudRGB::Ptr(new PointCloudRGB());
    mPath = path;
}

ColmapUtil::~ColmapUtil() {

}

void ColmapUtil::setup() {
    createDirectory(mPath + "images");
}

void ColmapUtil::update(std::map<std::string, std::shared_ptr<CameraModel>> &cameras, std::map<std::string, ImageType> &images) {
    for (auto &camera : cameras) {
        if ((mCamera.find(camera.first) == mCamera.end()) && (images.find(camera.first) != images.end())) {
            ColmapCamera colmap_camera;
            colmap_camera.camera_id  = mCameraIdx++;
            colmap_camera.model_id   = 1;
            colmap_camera.width      = images[camera.first].image.cols;
            colmap_camera.height     = images[camera.first].image.rows;
            Eigen::Matrix3d camera_K = camera.second->getCameraPara();
            colmap_camera.params.push_back(camera_K(0, 0));
            colmap_camera.params.push_back(camera_K(1, 1));
            colmap_camera.params.push_back(camera_K(0, 2));
            colmap_camera.params.push_back(camera_K(1, 2));
            mCamera[camera.first] = colmap_camera;
        }
    }

    for (auto &image : images) {
        if (mCamera.find(image.first) != mCamera.end()) {
            image.second.id = mImageIdx; // set id for each image

            ColmapImage colmap_image;
            colmap_image.camera_id = mCamera[image.first].camera_id;
            colmap_image.image_id  = mImageIdx;
            Eigen::Matrix4d Tcw    = cameras[image.first]->getPose().inverse();
            colmap_image.quat      = Eigen::Quaterniond(Tcw.topLeftCorner<3, 3>());
            colmap_image.trans     = Tcw.topRightCorner<3, 1>();
            colmap_image.image     = image.second.image;
            std::ostringstream image_name;
            image_name << std::internal << std::setfill('0') << std::setw(5) << mImageIdx++;
            colmap_image.image_name = image_name.str() + ".jpg";
            mImage.push_back(colmap_image);
        }
    }

    WriteImages(mPath, mImage);
}

void ColmapUtil::update(hash_map_3d<int64_t, Render::VoxelPtr> &map) {
    for (auto &it : map.m_map_3d_hash_map) {
        if (it.second->m_hits <= 2) { // remove some dynamic objects and noise
            continue;
        }

        for (auto &pts : it.second->m_pts) {
            if (pts.m_score < 0.1) {
                continue;
            }
            if (pts.m_tracks.size() <= 0) {
                continue;
            }

            {
                ColmapPoint3d p;
                p.point_id = mPointIdx++;
                p.pose     = Eigen::Vector3d(pts.m_pos[0], pts.m_pos[1], pts.m_pos[2]);
                p.color    = Eigen::Vector3d(pts.m_rgb[0], pts.m_rgb[1], pts.m_rgb[2]);
                p.tracks   = pts.m_tracks;
                mPoint3d.emplace_back(p);
            }

            {
                pcl::PointXYZRGB p;
                p.x = pts.m_pos[0];
                p.y = pts.m_pos[1];
                p.z = pts.m_pos[2];
                p.r = pts.m_rgb[0];
                p.g = pts.m_rgb[1];
                p.b = pts.m_rgb[2];
                mRGBPoints->points.push_back(p);
            }
        }
    }

    mRGBPoints->width = mRGBPoints->points.size();
    mRGBPoints->height = 1;
    LOG_INFO("COLMAP save {} points", mPoint3d.size());
}

void ColmapUtil::saveFiles() {
    WriteCamerasBinary(mPath, mCamera);
    WriteImagesBinary(mPath, mImage);
    WritePoints3DBinary(mPath, mPoint3d);
    pcl::io::savePCDFileBinary(mPath + "cloud.pcd", *mRGBPoints);
}