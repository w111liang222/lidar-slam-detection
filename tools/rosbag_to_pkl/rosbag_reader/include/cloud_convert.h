#pragma once

#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Types.h"

/**
 * 预处理雷达点云
 *
 * 将Velodyne, ouster, avia等数据转到FullCloud
 */
class CloudConvert {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class LidarType {
        AVIA = 1,  // 大疆的固态雷达
        VELO32,    // Velodyne 32线
        OUST64,    // ouster 64线
    };

    CloudConvert() = default;
    ~CloudConvert() = default;

    /**
     * 处理livox avia 点云
     * @param msg
     * @param pcl_out
     */
    void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloud::Ptr &pcl_out);

    /**
     * 处理sensor_msgs::PointCloud2点云
     * @param msg
     * @param pcl_out
     */
    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloud::Ptr &pcl_out);

    void init(int lidar_type, int num_scans, double time_scale);

   private:
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    PointCloud cloud_full_, cloud_out_;  // 输出点云
    LidarType lidar_type_ = LidarType::AVIA;     // 雷达类型
    int point_filter_num_ = 1;                   // 跳点
    int num_scans_ = 6;                          // 扫描线数
    float time_scale_ = 1e-3;                    // 雷达点的时间字段与秒的比例
};
