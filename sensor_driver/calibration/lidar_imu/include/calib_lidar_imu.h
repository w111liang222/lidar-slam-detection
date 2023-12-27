#pragma once

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/filters/voxel_grid.h>
#include <hdl_graph_slam/registrations.hpp>

using namespace std;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct LidarData
{
    double stamp;
    CloudT::Ptr cloud;
};

struct LidarFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp;
    Eigen::Matrix4d T;
    Eigen::Matrix4d gT;
    CloudT::Ptr cloud{nullptr};
};

struct ImuData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
    Eigen::Quaterniond rot;
};

class CalibLidarImu
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CalibLidarImu();
    ~CalibLidarImu();

    //@brief: add lidar data and calculate lidar odometry
    void addLidarData(const LidarData &data, Eigen::Matrix4d &global_T, Eigen::Matrix4d &delta_T);

    Eigen::Matrix4d get_lidar_T_R(const LidarData &data);

    const size_t getNumberOfScans() { return lidar_buffer_.size(); }

    //@brief: add imu data and cache
    void addImuData(const ImuData &data);

    const size_t getNumberOfImus() { return imu_buffer_.size(); }

    //@brief: integration imu data, align lidar odom and imu
    Eigen::Matrix3d calib(bool integration = false);

    std::vector<Eigen::Matrix4d> getLidarPose();

    std::vector<Eigen::Matrix4d> getImuPose();

private:
    //@brief: interpolated attitude from start attitude to end attitude by scale
    Eigen::Quaterniond getInterpolatedAttitude(const Eigen::Quaterniond &q_s_w, const Eigen::Quaterniond &q_e_w, double scale);

    //@brief: update relative transform between neighbor lidar frame by aligned imu data
    void optimize();

    //@brief: solve least square answer by constraints
    Eigen::Quaterniond solve(const vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> &corres);

    bool no_rtk_flag;
    Eigen::Matrix4d last_frame_pose_;

    vector<LidarFrame> lidar_buffer_;                                       // record relative transform between neighbor lidar frame
    vector<ImuData> imu_buffer_;                                            // record raw imu datas
    vector<pair<LidarFrame, Eigen::Quaterniond>> aligned_lidar_imu_buffer_; // aligned lidar frame and interpolated imu attitude at lidar stamp
    Eigen::Quaterniond q_l_b_;                                              // result

    CloudT::Ptr local_map_{nullptr};                                              // local map
    pcl::VoxelGrid<PointT> downer_;                                               // downsample local map
    pcl::Registration<PointT, PointT>::Ptr register_{nullptr}; // register object

    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres1_;
    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres2_;
};