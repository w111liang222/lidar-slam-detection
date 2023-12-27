// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <thread>

//pcl lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"

#include "mapping_types.h"

static LaserProcessingClass laserProcessing;
static RWQueue<PointCloudAttrImagePose*> pointCloudQueue;

void floamEdgeHandler(const PointCloud::Ptr &laserCloudMsg);
void floamSurfHandler(const PointCloud::Ptr &laserCloudMsg);
void floamInitGuessHandler(const Eigen::Matrix4d &initGuess);

void floamEnqueuePoints(PointCloudAttrPtr &points, Eigen::Matrix4d &init_guess)
{
    pointCloudQueue.enqueue(new PointCloudAttrImagePose(points, Eigen::Isometry3d(init_guess)));
}

void laser_processing(){
    while(1){
        PointCloudAttrImagePose *points = nullptr;
        pointCloudQueue.wait_dequeue(points);
        std::shared_ptr<PointCloudAttrImagePose> pointPtr = std::shared_ptr<PointCloudAttrImagePose>(points);

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

        laserProcessing.featureExtraction(pointPtr->points,pointcloud_edge,pointcloud_surf);

        pointcloud_edge->header.stamp = pointPtr->points->cloud->header.stamp;
        pointcloud_surf->header.stamp = pointPtr->points->cloud->header.stamp;
        floamEdgeHandler(pointcloud_edge);
        floamSurfHandler(pointcloud_surf);
        floamInitGuessHandler((pointPtr->T).matrix());
    }
}

void setLaserProcessingNode(int scan_line, double vertical_angle, double scan_period, double max_dis, double min_dis) {
    lidar::Lidar lidar_param_laser;
    lidar_param_laser.setScanPeriod(scan_period);
    lidar_param_laser.setVerticalAngle(vertical_angle);
    lidar_param_laser.setLines(scan_line);
    lidar_param_laser.setMaxDistance(max_dis);
    lidar_param_laser.setMinDistance(min_dis);

    laserProcessing.init(lidar_param_laser);
}

int initLaserProcessingNode()
{
    static bool isInited = false;
    if (isInited) {
        return 0;
    }
    isInited = true;
    static std::thread laser_processing_process{laser_processing};
    return 0;
}

