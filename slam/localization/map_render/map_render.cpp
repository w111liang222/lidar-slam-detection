#include "map_render.h"

#include <omp.h>
#include <unordered_set>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "Utils.h"
#include "InterProcess.h"

// #define DO_SAVE_COLMAP

using namespace Render;

void drawImage(cv::Mat &img, const double &u, const double &v, const float &depth) {
    float normalizedDepth = (depth - 0.5) / (20 - 0.5);
    int r = static_cast<int>(255 * (1 - normalizedDepth));
    int g = static_cast<int>(255 * normalizedDepth);
    int b = 0;
    r = std::max(0, std::min(255, r));
    g = std::max(0, std::min(255, g));
    b = std::max(0, std::min(255, b));
    cv::circle(img, cv::Point2f(u, v), 2, cv::Scalar(b, g, r));
}

// drawImage(im.second.image, pixel->u, pixel->v, pixel->depth);
// sensor_msgs::Image msg;
// fromCv(im.second.image, msg);
// PUBLISH_MSG(im.first, msg);


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
    ransac.setDistanceThreshold(0.2);
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

MapRender::MapRender() : mColmap("output/sparse/"){
    mActive = false;
    mLastPose = Eigen::Matrix4d::Identity();
}

MapRender::~MapRender() {
    mCameras.clear();
    mRenders.clear();
    mHashMap.clear();
}

void MapRender::setActive(bool active) {
    mActive = active;
    mColmap.setup();
}

void MapRender::setParameter(Eigen::Matrix4d staticTrans, std::map<std::string, CamParamType> cameraParam) {
    mStaticTrans  = staticTrans;
    mCameraParams = cameraParam;
    for (auto &it : mCameraParams) {
        it.second.staticTrans = it.second.staticTrans * mStaticTrans.inverse();
    }

    for (auto &it : mCameraParams) {
        mCameras[it.first] = std::make_shared<CameraModel>(it.second);
        mRenders[it.first] = create_image_render(it.second.K, it.second.staticTrans);
    }
}

void MapRender::updateMap(PointCloud::Ptr cloud, std::unordered_set<VoxelPtr> &voxel_hits) {
    for (int i = 0; i < cloud->points.size(); i++) {
        int64_t box_x = std::round(cloud->points[i].x * InvVoxelResolution);
        int64_t box_y = std::round(cloud->points[i].y * InvVoxelResolution);
        int64_t box_z = std::round(cloud->points[i].z * InvVoxelResolution);
        hash_point hash(box_x, box_y, box_z);

        VoxelPtr voxel;
        auto it = mHashMap.m_map_3d_hash_map.find(hash);
        if(it == mHashMap.m_map_3d_hash_map.end()) {
            voxel = std::make_shared<Voxel>();
            voxel->m_cx = int64_t((box_x * VoxelResolution) * InvPointResolution);
            voxel->m_cy = int64_t((box_y * VoxelResolution) * InvPointResolution);
            voxel->m_cz = int64_t((box_z * VoxelResolution) * InvPointResolution);
            mHashMap.m_map_3d_hash_map.emplace(hash, voxel);
        } else {
            voxel = it->second;
        }

        int32_t offset_x = int64_t(cloud->points[i].x * InvPointResolution) - voxel->m_cx + 5;
        int32_t offset_y = int64_t(cloud->points[i].y * InvPointResolution) - voxel->m_cy + 5;
        int32_t offset_z = int64_t(cloud->points[i].z * InvPointResolution) - voxel->m_cz + 5;
        int32_t offset = offset_x * 100 + offset_y * 10 + offset_z;

        if (voxel->m_set.find(offset) == voxel->m_set.end()) {
            voxel->m_set.insert(offset);
            voxel->m_pts.emplace_back(cloud->points[i].getVector3fMap());
        }
        voxel_hits.insert(voxel);
    }
}

void MapRender::render(Eigen::Matrix4d pose, PointCloudAttrPtr cloud, std::map<std::string, ImageType> images) {
    if (!mActive) {
        return;
    }

    // movement detection
    Eigen::Isometry3d delta = Eigen::Isometry3d(mLastPose.inverse() * pose);
    if (delta.translation().norm() < 0.1) {
        return;
    }
    mLastPose = pose;

    if (!detect_ground(cloud->cloud)) {
        return;
    }

    // transform to world coordinate
    PointCloud::Ptr world_cloud(new PointCloud());
    pcl::transformPointCloud(*cloud->cloud, *world_cloud, pose);

    // update world voxels
    std::unordered_set<VoxelPtr> voxel_hits_set;
    updateMap(world_cloud, voxel_hits_set);

    std::vector<VoxelPtr> voxel_hits(voxel_hits_set.begin(), voxel_hits_set.end());
    int num_voxel_hit = voxel_hits.size();
    for (int i = 0; i < num_voxel_hit; ++i) {
        ++voxel_hits[i]->m_hits;
    }

    // update camera
    for (auto &im : images) {
        imageCvtColor(im.second.image);
        mCameras[im.first]->setPose(im.second.T);
        mRenders[im.first]->setPose(im.second.T, im.second.image.cols, im.second.image.rows);
    }

#ifdef DO_SAVE_COLMAP
    mColmap.update(mCameras, images);
#endif

    int image_num = images.size();
    std::vector<std::string>    image_names(image_num);
    std::vector<ImageType>      image_datas(image_num);
    std::transform(images.begin(), images.end(), image_names.begin(), [](auto pair){return pair.first;});
    std::transform(images.begin(), images.end(), image_datas.begin(), [](auto pair){return pair.second;});
    // project 3D points
    #pragma omp parallel for
    for (int n = 0; n < image_num; n++) {
        int cols = image_datas[n].image.cols;
        int rows = image_datas[n].image.rows;
        std::vector<VoxelPixel> voxel_pixel(num_voxel_hit);

        #pragma omp parallel for
        for (int i = 0; i < num_voxel_hit; ++i) {
            double u, v;
            vec_3 pt_cam, pt_w(voxel_hits[i]->m_cx * PointResolution, voxel_hits[i]->m_cy * PointResolution, voxel_hits[i]->m_cz * PointResolution);
            if (!mCameras[image_names[n]]->project3DPoints(pt_w, pt_cam, u, v)) {
                continue;
            }
            if (!mCameras[image_names[n]]->point2DAvailable(cols, rows, u, v)) {
                continue;
            }
            if (pt_cam[2] <= MinDepth || pt_cam[2] >= MaxDepth) {
                continue;
            }

            double max_u = u, min_u = u;
            double max_v = v, min_v = v;
            voxel_pixel[i].depth = pt_cam[2];
            voxel_pixel[i].pixel.resize(voxel_hits[i]->m_pts.size(), nullptr);
            for (int j = 0; j < voxel_hits[i]->m_pts.size(); ++j) {
                double u, v;
                vec_3 pt_cam, pt_w(voxel_hits[i]->m_pts[j].m_pos[0], voxel_hits[i]->m_pts[j].m_pos[1], voxel_hits[i]->m_pts[j].m_pos[2]);
                if (!mCameras[image_names[n]]->project3DPoints(pt_w, pt_cam, u, v)) {
                    continue;
                }
                if (!mCameras[image_names[n]]->point2DAvailable(cols, rows, u, v)) {
                    continue;
                }

                max_u = std::max(max_u, u);
                max_v = std::max(max_v, v);
                min_u = std::min(min_u, u);
                min_v = std::min(min_v, v);
                voxel_pixel[i].pixel[j]        = std::make_shared<PixelType>();
                voxel_pixel[i].pixel[j]->voxel = voxel_hits[i];
                voxel_pixel[i].pixel[j]->idx   = j;
                voxel_pixel[i].pixel[j]->depth = pt_cam[2];
                voxel_pixel[i].pixel[j]->u     = u;
                voxel_pixel[i].pixel[j]->v     = v;
                voxel_pixel[i].pixel[j]->score = 1.0 - std::abs(u - cols / 2.0) / (cols / 2.0);
                voxel_pixel[i].pixel[j]->bgr   = getSubPixel<cv::Vec3b>(image_datas[n].image, v, u);
                voxel_pixel[i].pixel[j]->track.image_id = image_datas[n].id;
            }

            // the few area covered by the voxel, mask as invalid
            if ((max_u - min_u) <= 10.0 && (max_v - min_v) <= 10.0) {
                voxel_pixel[i].depth = -1.0;
            }
        }

        // check occlusion
        std::unordered_map<hash_uv, PixelPtr> pixels;
        std::vector<float> depths(rows * cols, -1.0);
        for (int i = 0; i < num_voxel_hit; ++i) {
            if (voxel_pixel[i].depth > 0) {
                for (int j = 0; j < voxel_pixel[i].pixel.size(); ++j) {
                    if (voxel_pixel[i].pixel[j]) {
                        int u  = int(std::round(voxel_pixel[i].pixel[j]->u));
                        int v  = int(std::round(voxel_pixel[i].pixel[j]->v));
                        int uv = v * cols + u;
                        if ((voxel_pixel[i].pixel[j]->depth > 0) && (depths[uv] < 0 || depths[uv] > voxel_pixel[i].pixel[j]->depth)) {
                            depths[uv] = voxel_pixel[i].pixel[j]->depth;
                            pixels[hash_uv(u, v)] = voxel_pixel[i].pixel[j];
                        }
                    }
                }
            }
        }

        // check depth continuous
        mRenders[image_names[n]]->checkDepthContinuous(pixels, depths);
    }

    for (int n = 0; n < image_num; n++) {
        mRenders[image_names[n]]->updateVoxel();
    }
}

void MapRender::getColorMap(PointCloudRGB::Ptr& points) {
#ifdef DO_SAVE_COLMAP
    mColmap.update(mHashMap);
    mColmap.saveFiles();
#endif
    for (auto &it : mHashMap.m_map_3d_hash_map) {
        if (it.second->m_hits <= 2) { // remove some dynamic objects and noise
            continue;
        }

        for (auto &pts : it.second->m_pts) {
            if (pts.m_score < 0.1) {
                continue;
            }
            pcl::PointXYZRGB p;
            p.x = pts.m_pos[0];
            p.y = pts.m_pos[1];
            p.z = pts.m_pos[2];
            p.r = pts.m_rgb[0];
            p.g = pts.m_rgb[1];
            p.b = pts.m_rgb[2];
            points->points.push_back(p);
        }
    }
    points->width = points->points.size();
    points->height = 1;
}

void MapRender::clear() {
    mHashMap.clear();
}