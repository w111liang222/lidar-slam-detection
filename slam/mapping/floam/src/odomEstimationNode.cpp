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
#include "odomEstimationClass.h"

#include "mapping_types.h"

static OdomEstimationClass odomEstimation;
static RWQueue<PointCloudType*> pointCloudEdgeBuf;
static RWQueue<PointCloudType*> pointCloudSurfBuf;
static RWQueue<Eigen::Matrix4d> initGuessBuf;

static RWQueue<Eigen::Isometry3d> laserOdometryBuf;

void floamSurfHandler(const PointCloud::Ptr &laserCloudMsg)
{
    pointCloudSurfBuf.enqueue(new PointCloudType(laserCloudMsg));
}
void floamEdgeHandler(const PointCloud::Ptr &laserCloudMsg)
{
    pointCloudEdgeBuf.enqueue(new PointCloudType(laserCloudMsg));
}
void floamInitGuessHandler(const Eigen::Matrix4d &initGuess)
{
    initGuessBuf.enqueue(initGuess);
}

void getFloamOdom(Eigen::Isometry3d &odom) {
    laserOdometryBuf.wait_dequeue(odom);
}

bool is_odom_inited = false;
void odom_estimation(){
    while(1){
        //read data
        PointCloudType *pointsEdge = nullptr;
        PointCloudType *pointsSurf = nullptr;
        Eigen::Matrix4d initGuess;
        pointCloudEdgeBuf.wait_dequeue(pointsEdge);
        pointCloudSurfBuf.wait_dequeue(pointsSurf);
        initGuessBuf.wait_dequeue(initGuess);
        std::shared_ptr<PointCloudType> pointsEdgePtr = std::shared_ptr<PointCloudType>(pointsEdge);
        std::shared_ptr<PointCloudType> pointsSurfPtr = std::shared_ptr<PointCloudType>(pointsSurf);

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
        pointcloud_edge_in = pointsEdgePtr->points;
        pointcloud_surf_in = pointsSurfPtr->points;

        if(is_odom_inited == false){
            odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
            is_odom_inited = true;
        }else{
            odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, initGuess);
        }

        // publish odometry
        laserOdometryBuf.enqueue(odomEstimation.odom);
    }
}

void setOdomEstimationNode(int scan_line, double vertical_angle, double scan_period, double max_dis, double min_dis, double map_resolution) {
    lidar::Lidar lidar_param_odom;
    is_odom_inited = false;
    lidar_param_odom.setScanPeriod(scan_period);
    lidar_param_odom.setVerticalAngle(vertical_angle);
    lidar_param_odom.setLines(scan_line);
    lidar_param_odom.setMaxDistance(max_dis);
    lidar_param_odom.setMinDistance(min_dis);

    odomEstimation.init(lidar_param_odom, map_resolution);
}

int initOdomEstimationNode()
{
    static bool isInited = false;
    if (isInited) {
        return 0;
    }
    isInited = true;

    static std::thread odom_estimation_process{odom_estimation};
    return 0;
}

