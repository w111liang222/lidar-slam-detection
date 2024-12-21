// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
// #include <csignal>
#include <unistd.h>
// #include <Python.h>
#include <so3_math.h>
// #include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Vector3.h>
// #include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <ivox3d/ivox3d.h>

#include "mapping_types.h"
#include "slam_utils.h"

// #define USE_IKD_TREE
#define USE_IVOX

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

using namespace faster_lio;

/*** Time Log Variables ***/
// double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
// double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
// double match_time = 0, solve_time = 0, solve_const_H_time = 0;
// int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, extrinsic_est_en = false;
bool   degenerate_detect_en = true, wheelspeed_en = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

// string root_dir = ROOT_DIR;

double res_mean_last = 0.05, total_residual = 0.0;
double travel_distance = 0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_ins = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0;
int    feats_down_size = 0, NUM_MAX_ITERATIONS = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
bool   is_degenerate = false;
// bool   scan_pub_en = false, scan_body_pub_en = false;

// vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<ImuType> imu_buffer;
deque<RTKType> ins_buffer;

// PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;
using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
std::shared_ptr<IVoxType> ivox(nullptr);

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
// V3D euler_cur;
// V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 last_pos_lid, pos_lid;

// nav_msgs::Odometry odomAftMapped;
// geometry_msgs::Quaternion geoQuat;
// geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// void SigHandle(int sig)
// {
//     flg_exit = true;
//     ROS_WARN("catch sig %d", sig);
//     sig_buffer.notify_all();
// }

// inline void dump_lio_state_to_log(FILE *fp)
// {
//     V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
//     fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
//     fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
//     fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos
//     fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
//     fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
//     fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
//     fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g
//     fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a
//     fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
//     fprintf(fp, "\r\n");
//     fflush(fp);
// }

// void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
// {
//     V3D p_body(pi->x, pi->y, pi->z);
//     V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

//     po->x = p_global(0);
//     po->y = p_global(1);
//     po->z = p_global(2);
//     po->intensity = pi->intensity;
// }


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    // kdtree_delete_counter = 0;
    // kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) ikdtree.Delete_Point_Boxes(cub_needrm);
    // kdtree_delete_time = omp_get_wtime() - delete_begin;
}

// void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     mtx_buffer.lock();
//     scan_count ++;
//     double preprocess_start_time = omp_get_wtime();
//     if (msg->header.stamp.toSec() < last_timestamp_lidar)
//     {
//         ROS_ERROR("lidar loop back, clear buffer");
//         lidar_buffer.clear();
//     }

//     PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
//     p_pre->process(msg, ptr);
//     lidar_buffer.push_back(ptr);
//     time_buffer.push_back(msg->header.stamp.toSec());
//     last_timestamp_lidar = msg->header.stamp.toSec();
//     // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }

void fastlio_pcl_enqueue(PointCloudAttrPtr &points)
{
    mtx_buffer.lock();
    // scan_count ++;
    // double preprocess_start_time = omp_get_wtime();
    // if ((points->header.stamp / 1000000.0) < last_timestamp_lidar)
    // {
    //     LOG_ERROR("lidar loop back, clear buffer");
    //     lidar_buffer.clear();
    // }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(points, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(points->cloud->header.stamp / 1000000.0);
    last_timestamp_lidar = points->cloud->header.stamp / 1000000.0;
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// double timediff_lidar_wrt_imu = 0.0;
// bool   timediff_set_flg = false;
// void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
// {
//     mtx_buffer.lock();
//     double preprocess_start_time = omp_get_wtime();
//     scan_count ++;
//     if (msg->header.stamp.toSec() < last_timestamp_lidar)
//     {
//         ROS_ERROR("lidar loop back, clear buffer");
//         lidar_buffer.clear();
//     }
//     last_timestamp_lidar = msg->header.stamp.toSec();

//     if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
//     {
//         printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
//     }

//     if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
//     {
//         timediff_set_flg = true;
//         timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
//         printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
//     }

//     PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
//     p_pre->process(msg, ptr);
//     lidar_buffer.push_back(ptr);
//     time_buffer.push_back(last_timestamp_lidar);

//     s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }

// void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
// {
//     publish_count ++;
//     // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
//     sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

//     if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
//     {
//         msg->header.stamp = \
//         ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
//     }

//     double timestamp = msg->header.stamp.toSec();

//     mtx_buffer.lock();

//     if (timestamp < last_timestamp_imu)
//     {
//         ROS_WARN("imu loop back, clear buffer");
//         imu_buffer.clear();
//     }

//     last_timestamp_imu = timestamp;

//     imu_buffer.push_back(msg);
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }

void fastlio_imu_enqueue(ImuType imu)
{
    double timestamp = imu.stamp;

    mtx_buffer.lock();

    // if (timestamp < last_timestamp_imu)
    // {
    //     ROS_WARN("imu loop back, clear buffer");
    //     imu_buffer.clear();
    // }

    last_timestamp_imu = timestamp;
    imu.acc = imu.acc / 9.81;

    imu_buffer.push_back(imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void fastlio_ins_enqueue(bool rtk_valid, RTKType ins) {
    // check LLA or Wheel is avaliable
    if (!rtk_valid && ins.sensor.find("Wheel") == std::string::npos) {
        return;
    }

    mtx_buffer.lock();

    last_timestamp_ins = ins.timestamp / 1000000.0;

    Eigen::Vector3d vec(ins.Ve, ins.Vn, ins.Vu);
    // ENU -> Ego(INS)
    Eigen::Matrix4d Tve = getTransformFromRPYT(0, 0, 0, -ins.heading, ins.pitch, ins.roll).inverse();
    vec = Tve.topLeftCorner<3, 3>() * vec;
    // Ego(INS) -> IMU
    vec = Lidar_R_wrt_IMU * vec;

    ins.Ve = vec[0];
    ins.Vn = vec[1];
    ins.Vu = 0.0; // vec[2]; body up speed of INS is not accurate

    ins_buffer.push_back(ins);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lck(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        // if (meas.lidar->points.size() <= 1) // time too little
        // {
        //     lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        //     LOG_WARN("Too few input point cloud!");
        // }
        // else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        // {
        //     lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        // }
        // else
        // {
        //     scan_num ++;
        //     lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        //     lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        // }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    // if (last_timestamp_imu < lidar_end_time)
    // {
    //     return false;
    // }

    /*** push imu data, and pop from imu buffer ***/
    meas.imu.clear();
    while(!imu_buffer.empty()) {
        double imu_time = imu_buffer.front().stamp;
        if (imu_time > lidar_end_time) {
            break;
        }
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    meas.ins.clear();
    while(!ins_buffer.empty()) {
        double ins_time = ins_buffer.front().timestamp / 1000000.0;
        if (ins_time > lidar_end_time) {
            break;
        }
        meas.ins.push_back(ins_buffer.front());
        ins_buffer.pop_front();
    }

    // /*** push imu data, and pop from imu buffer ***/
    // double imu_time = imu_buffer.front().stamp;
    // meas.imu.clear();
    // while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    // {
    //     imu_time = imu_buffer.front().stamp;
    //     if(imu_time > lidar_end_time) break;
    //     meas.imu.push_back(imu_buffer.front());
    //     imu_buffer.pop_front();
    // }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

// int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    // double st_time = omp_get_wtime();
    // add_point_size = ikdtree.Add_Points(PointToAdd, true);
#if defined(USE_IKD_TREE)
    ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
#elif defined(USE_IVOX)
    ivox->AddPoints(PointToAdd, travel_distance);
    ivox->AddPoints(PointNoNeedDownsample, travel_distance);
#endif
    // add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    // kdtree_incremental_time = omp_get_wtime() - st_time;
}

// PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
// PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
// void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
// {
//     if(scan_pub_en)
//     {
//         PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
//         int size = laserCloudFullRes->points.size();
//         PointCloudXYZI::Ptr laserCloudWorld( \
//                         new PointCloudXYZI(size, 1));

//         for (int i = 0; i < size; i++)
//         {
//             RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
//                                 &laserCloudWorld->points[i]);
//         }

//         sensor_msgs::PointCloud2 laserCloudmsg;
//         pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
//         laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
//         laserCloudmsg.header.frame_id = "camera_init";
//         pubLaserCloudFull.publish(laserCloudmsg);
//         publish_count -= PUBFRAME_PERIOD;
//     }

//     /**************** save map ****************/
//     /* 1. make sure you have enough memories
//     /* 2. noted that pcd save will influence the real-time performences **/
//     if (pcd_save_en)
//     {
//         int size = feats_undistort->points.size();
//         PointCloudXYZI::Ptr laserCloudWorld( \
//                         new PointCloudXYZI(size, 1));

//         for (int i = 0; i < size; i++)
//         {
//             RGBpointBodyToWorld(&feats_undistort->points[i], \
//                                 &laserCloudWorld->points[i]);
//         }
//         *pcl_wait_save += *laserCloudWorld;

//         static int scan_wait_num = 0;
//         scan_wait_num ++;
//         if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
//         {
//             pcd_index ++;
//             string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
//             pcl::PCDWriter pcd_writer;
//             cout << "current scan saved to /PCD/" << all_points_dir << endl;
//             pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
//             pcl_wait_save->clear();
//             scan_wait_num = 0;
//         }
//     }
// }

// void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
// {
//     int size = feats_undistort->points.size();
//     PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

//     for (int i = 0; i < size; i++)
//     {
//         RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
//                             &laserCloudIMUBody->points[i]);
//     }

//     sensor_msgs::PointCloud2 laserCloudmsg;
//     pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
//     laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
//     laserCloudmsg.header.frame_id = "body";
//     pubLaserCloudFull_body.publish(laserCloudmsg);
//     publish_count -= PUBFRAME_PERIOD;
// }

// void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
// {
//     PointCloudXYZI::Ptr laserCloudWorld( \
//                     new PointCloudXYZI(effct_feat_num, 1));
//     for (int i = 0; i < effct_feat_num; i++)
//     {
//         RGBpointBodyToWorld(&laserCloudOri->points[i], \
//                             &laserCloudWorld->points[i]);
//     }
//     sensor_msgs::PointCloud2 laserCloudFullRes3;
//     pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
//     laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
//     laserCloudFullRes3.header.frame_id = "camera_init";
//     pubLaserCloudEffect.publish(laserCloudFullRes3);
// }

// void publish_map(const ros::Publisher & pubLaserCloudMap)
// {
//     sensor_msgs::PointCloud2 laserCloudMap;
//     pcl::toROSMsg(*featsFromMap, laserCloudMap);
//     laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
//     laserCloudMap.header.frame_id = "camera_init";
//     pubLaserCloudMap.publish(laserCloudMap);
// }

// template<typename T>
// void set_posestamp(T & out)
// {
//     out.pose.position.x = state_point.pos(0);
//     out.pose.position.y = state_point.pos(1);
//     out.pose.position.z = state_point.pos(2);
//     out.pose.orientation.x = geoQuat.x;
//     out.pose.orientation.y = geoQuat.y;
//     out.pose.orientation.z = geoQuat.z;
//     out.pose.orientation.w = geoQuat.w;

// }


void fastlio_odometry(Eigen::Matrix4d &odom_s, Eigen::Matrix4d &odom_e)
{
    odom_s = Eigen::Matrix4d::Identity();
    odom_e = Eigen::Matrix4d::Identity();

    odom_s.topRightCorner<3, 1>() = Eigen::Vector3d(p_imu->start_state_point.pos(0),
                                                    p_imu->start_state_point.pos(1),
                                                    p_imu->start_state_point.pos(2));
    odom_s.topLeftCorner<3, 3>()  = Eigen::Quaterniond(p_imu->start_state_point.rot.coeffs()[3],
                                                       p_imu->start_state_point.rot.coeffs()[0],
                                                       p_imu->start_state_point.rot.coeffs()[1],
                                                       p_imu->start_state_point.rot.coeffs()[2]).normalized().toRotationMatrix();

    odom_e.topRightCorner<3, 1>() = Eigen::Vector3d(state_point.pos(0),
                                                    state_point.pos(1),
                                                    state_point.pos(2));
    odom_e.topLeftCorner<3, 3>()  = Eigen::Quaterniond(state_point.rot.coeffs()[3],
                                                       state_point.rot.coeffs()[0],
                                                       state_point.rot.coeffs()[1],
                                                       state_point.rot.coeffs()[2]).normalized().toRotationMatrix();
}

std::vector<double> fastlio_state()
{
    std::vector<double> state;
    state.push_back(p_imu->start_state_point.pos(0));
    state.push_back(p_imu->start_state_point.pos(1));
    state.push_back(p_imu->start_state_point.pos(2));
    state.push_back(p_imu->start_state_point.rot.coeffs()[0]);
    state.push_back(p_imu->start_state_point.rot.coeffs()[1]);
    state.push_back(p_imu->start_state_point.rot.coeffs()[2]);
    state.push_back(p_imu->start_state_point.rot.coeffs()[3]);
    state.push_back(p_imu->start_state_point.vel(0));
    state.push_back(p_imu->start_state_point.vel(1));
    state.push_back(p_imu->start_state_point.vel(2));
    state.push_back(p_imu->start_state_point.ba(0));
    state.push_back(p_imu->start_state_point.ba(1));
    state.push_back(p_imu->start_state_point.ba(2));
    state.push_back(p_imu->start_state_point.bg(0));
    state.push_back(p_imu->start_state_point.bg(1));
    state.push_back(p_imu->start_state_point.bg(2));
    state.push_back(p_imu->start_state_point.grav[0]);
    state.push_back(p_imu->start_state_point.grav[1]);
    state.push_back(p_imu->start_state_point.grav[2]);
    state.push_back(p_imu->mean_acc_norm);
    return state;
}

bool fastlio_is_init()
{
    return p_imu->IsInit();
}

// void publish_odometry(const ros::Publisher & pubOdomAftMapped)
// {
//     odomAftMapped.header.frame_id = "camera_init";
//     odomAftMapped.child_frame_id = "body";
//     odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
//     set_posestamp(odomAftMapped.pose);
//     pubOdomAftMapped.publish(odomAftMapped);
//     auto P = kf.get_P();
//     for (int i = 0; i < 6; i ++)
//     {
//         int k = i < 3 ? i + 3 : i - 3;
//         odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
//         odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
//         odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
//         odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
//         odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
//         odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
//     }

//     static tf::TransformBroadcaster br;
//     tf::Transform                   transform;
//     tf::Quaternion                  q;
//     transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
//                                     odomAftMapped.pose.pose.position.y, \
//                                     odomAftMapped.pose.pose.position.z));
//     q.setW(odomAftMapped.pose.pose.orientation.w);
//     q.setX(odomAftMapped.pose.pose.orientation.x);
//     q.setY(odomAftMapped.pose.pose.orientation.y);
//     q.setZ(odomAftMapped.pose.pose.orientation.z);
//     transform.setRotation( q );
//     br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
// }

// void publish_path(const ros::Publisher pubPath)
// {
//     set_posestamp(msg_body_pose);
//     msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
//     msg_body_pose.header.frame_id = "camera_init";

//     /*** if path is too large, the rvis will crash ***/
//     static int jjj = 0;
//     jjj++;
//     if (jjj % 10 == 0)
//     {
//         path.poses.push_back(msg_body_pose);
//         pubPath.publish(path);
//     }
// }

void h_share_model_wheelspeed(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    ekfom_data.valid = false;
    // check timestamp synchronous within 10 ms
    if (wheelspeed_en && !Measures.ins.empty() && (Measures.lidar_end_time - Measures.ins.back().timestamp / 1000000.0) < 0.01)
    {
        // velocity from body to world
        V3D vel = s.rot * V3D(Measures.ins.back().Ve, Measures.ins.back().Vn, Measures.ins.back().Vu);

        ekfom_data.h_x = Eigen::MatrixXd::Zero(3, 15);
        ekfom_data.h.resize(3);
        // measurement res
        ekfom_data.h = vel - s.vel;
        // dh/dv
        ekfom_data.h_x.block<3, 3>(0, 12) = Eigen::MatrixXd::Identity(3, 3);
        ekfom_data.valid = true;
    }
}

void h_share_model_geometric(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    // double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
#if defined(USE_IKD_TREE)
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
#elif defined(USE_IVOX)
            ivox->GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS, 5);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : true;
#endif
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        LOG_WARN("No Effective Points!");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    // match_time  += omp_get_wtime() - match_start;
    // double solve_start_  = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 15); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 15>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C), 0.0, 0.0, 0.0;
        }
        else
        {
            ekfom_data.h_x.block<1, 15>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

    if (degenerate_detect_en)
    {
        // degenerate detection
        is_degenerate = false;
        Eigen::MatrixXd h_x = ekfom_data.h_x.leftCols(3);
        Eigen::Matrix<double, 3, 3> HTH = h_x.transpose() * h_x;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> esolver(HTH);
        Eigen::Matrix<double, 3, 1> mat_e  = esolver.eigenvalues().real();
        Eigen::Matrix<double, 3, 3> mat_v  = esolver.eigenvectors().real();
        Eigen::Matrix<double, 3, 3> mat_v2 = mat_v.transpose();

        for (int i = 0; i < 3; i++)
        {
            // calculate the feature's contribution to eigen vector
            float local_contri = 0, local_strong = 0;
            for (int j = 0; j < effct_feat_num; j++)
            {
                V3D feat_row = h_x.row(j).transpose();
                feat_row.normalize();
                V3D dir = mat_v.col(i);
                const float dotp = fabs(feat_row.dot(dir));
                if (dotp > 0.1736) // cos(80)
                {
                    local_contri += dotp;
                }
                if (dotp > 0.7070) // cos(45)
                {
                    local_strong += dotp;
                }
            }
            if (local_contri < 250.0 && local_strong < 50.0)
            {
                for (int j = 0; j < 3; j++)
                {
                    mat_v2(i, j) = 0;
                }
                is_degenerate = true;
            }
        }

        if (is_degenerate)
        {
            Eigen::Matrix<double, 3, 3> mat_p = mat_v.transpose().inverse() * mat_v2;
            ekfom_data.h_x.leftCols(3) = (mat_p * h_x.transpose()).transpose();
        }
    }
    // solve_time += omp_get_wtime() - solve_start_;
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    // Calculate WheelSpeed Terms
    esekfom::dyn_share_datastruct<double> ekfom_data_wheelspeed;
    h_share_model_wheelspeed(s, ekfom_data_wheelspeed);

    // Calculate Point-to-Plane Terms
    esekfom::dyn_share_datastruct<double> ekfom_data_geo = ekfom_data;
    h_share_model_geometric(s, ekfom_data_geo);

    int n_terms = ekfom_data_geo.h.rows() + ekfom_data_wheelspeed.h.rows();
    if (ekfom_data_wheelspeed.valid)
    {
        float weight_wheelspeed = 0.0;
        if (!is_degenerate)
        {
            weight_wheelspeed = 0.0001 * ekfom_data_geo.h.rows();
        }
        else
        {
            weight_wheelspeed = 0.001 * ekfom_data_geo.h.rows();
        }

        // Concatenate Geometric and Wheel speed Terms
        ekfom_data.h_x = Eigen::MatrixXd::Zero(n_terms, 15);
        ekfom_data.h_x << ekfom_data_geo.h_x, ekfom_data_wheelspeed.h_x;
        ekfom_data.h = Eigen::VectorXd::Zero(n_terms);
        ekfom_data.h << ekfom_data_geo.h, ekfom_data_wheelspeed.h * weight_wheelspeed;
    }
    else
    {
        ekfom_data.h_x = ekfom_data_geo.h_x;
        ekfom_data.h = ekfom_data_geo.h;
    }

    if (n_terms == 0)
    {
        ekfom_data.valid = false;
    }
}

int fastlio_init(vector<double> &extT, vector<double>& extR, int filter_num, int max_point_num, double scan_period, bool undistort) {
    NUM_MAX_ITERATIONS = 4;
    filter_size_surf_min = 0.5;
    filter_size_map_min = 0.5;
    cube_len = 1000;
    DET_RANGE = 300.0;
    fov_deg = 180;

    travel_distance = 0;
    last_timestamp_lidar = 0;
    last_timestamp_imu = -1.0;
    last_timestamp_ins = -1.0;
    lidar_end_time = 0;
    first_lidar_time = 0.0;
    effct_feat_num = 0;
    feats_down_size = 0;
    lidar_pushed = false;
    flg_first_scan = true;
    flg_EKF_inited = false;
    is_degenerate = false;

    Nearest_Points.clear();
    time_buffer.clear();
    lidar_buffer.clear();
    imu_buffer.clear();
    ins_buffer.clear();

    feats_undistort = PointCloudXYZI::Ptr(new PointCloudXYZI());
    feats_down_body = PointCloudXYZI::Ptr(new PointCloudXYZI());
    feats_down_world =PointCloudXYZI::Ptr(new PointCloudXYZI());
    normvec          =PointCloudXYZI::Ptr(new PointCloudXYZI(100000, 1));
    laserCloudOri    =PointCloudXYZI::Ptr(new PointCloudXYZI(100000, 1));
    corr_normvect    =PointCloudXYZI::Ptr(new PointCloudXYZI(100000, 1));

    ikdtree.Build(PointVector());
    IVoxType::Options ivox_options;
    ivox_options.resolution_ = 0.5;
    ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY74; // switch to NEARBY18 after 1.0s
    ivox_options.capacity_ = 100000;
    ivox_options.max_distance_ = 100.0;
    ivox = std::make_shared<IVoxType>(ivox_options);

    XAxisPoint_body = V3F(LIDAR_SP_LEN, 0.0, 0.0);
    XAxisPoint_world= V3F(LIDAR_SP_LEN, 0.0, 0.0);

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    gyr_cov = 0.1;
    acc_cov = 0.1;
    b_gyr_cov = 0.0001;
    b_acc_cov = 0.0001;

    extrinT = extT;
    extrinR = extR;
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    p_pre.reset(new Preprocess());
    p_imu.reset(new ImuProcess());

    p_pre->blind = 0.1;
    p_pre->N_SCANS = 16;
    p_pre->SCAN_RATE = 10;
    p_pre->point_filter_num = filter_num;
    p_pre->max_point_num = max_point_num;
    p_pre->feature_enabled = false;

    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->undistort = undistort;

    Measures = MeasureGroup();
    kf = esekfom::esekf<state_ikfom, 12, input_ikfom>();
    state_point = state_ikfom();
    last_pos_lid = vect3();
    pos_lid = vect3();

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    LocalMap_Points = BoxPointType();
    Localmap_Initialized = false;

    lidar_mean_scantime = scan_period;
    scan_num = 0;
    return 0;
}

bool fastlio_main()
{
    // ros::NodeHandle nh;

    /*** debug record ***/
    // FILE *fp;
    // string pos_log_dir = root_dir + "/Log/pos_log.txt";
    // fp = fopen(pos_log_dir.c_str(),"w");

    // ofstream fout_pre, fout_out, fout_dbg;
    // fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    // fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    // fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    // if (fout_pre && fout_out)
    //     cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    // else
    //     cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    /* ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
         nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \*/
    // ros::Subscriber sub_pcl = nh.subscribe("/lidar0OusterOS1128", 200000, standard_pcl_cbk);
    // ros::Subscriber sub_imu = nh.subscribe("/imu_raw", 200000, imu_cbk);
    // ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_registered", 100000);
    // ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_registered_body", 100000);
    // ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_effected", 100000);
    // ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/Laser_map", 100000);
    // ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
    //         ("/Odometry", 100000);
    // ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
    //         ("/path", 100000);
//------------------------------------------------------------------------------------------------------
    // signal(SIGINT, SigHandle);
    // ros::Rate rate(5000);
    // bool status = ros::ok();
    // while (status)
    // {
        // if (flg_exit) break;
        // ros::spinOnce();
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return true;
            }

            // double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            // match_time = 0;
            // kdtree_search_time = 0.0;
            // solve_time = 0;
            // solve_const_H_time = 0;
            // svd_time   = 0;
            // t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                LOG_WARN("FastLio undistort points is empty");
                return true;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
#if defined(USE_IKD_TREE)
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
#endif

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            // t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
#if defined(USE_IKD_TREE)
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return true;
            }
#elif defined(USE_IVOX)
            if(!ivox->NumValidGrids())
            {
                if(feats_down_size > 5)
                {
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ivox->AddPoints(feats_down_world->points, travel_distance);
                }
                return true;
            }

            if (ivox->GetNearByType() != IVoxType::NearbyType::NEARBY18 && (Measures.lidar_beg_time - first_lidar_time) > 10 * INIT_TIME) {
                ivox->SetNearByType(IVoxType::NearbyType::NEARBY18);
            }
#endif
            // int featsFromMapNum = ikdtree.validnum();
            // kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                LOG_WARN("FastLio downsampled feature size is too small!");
                return true;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            // fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            // if(0) // If you need to see map point, change to "if(1)"
            // {
            //     PointVector ().swap(ikdtree.PCL_Storage);
            //     ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            //     featsFromMap->clear();
            //     featsFromMap->points = ikdtree.PCL_Storage;
            // }

            // pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            // t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            // double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            if (is_degenerate)
            {
                LOG_WARN("FastLio is degenerate!");
            }
            state_point = kf.get_x();
            // euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            travel_distance = travel_distance + (pos_lid - last_pos_lid).norm();
            last_pos_lid = pos_lid;
            // geoQuat.x = state_point.rot.coeffs()[0];
            // geoQuat.y = state_point.rot.coeffs()[1];
            // geoQuat.z = state_point.rot.coeffs()[2];
            // geoQuat.w = state_point.rot.coeffs()[3];

            // double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            // publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            // t3 = omp_get_wtime();
            map_incremental();
            // t5 = omp_get_wtime();

            /******* Publish points *******/
            // if (path_en)                         publish_path(pubPath);
            // publish_frame_world(pubLaserCloudFull);
            // publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            // if (runtime_pos_log)
            // {
            //     frame_num ++;
            //     kdtree_size_end = ikdtree.size();
            //     aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            //     aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
            //     aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
            //     aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
            //     aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
            //     aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
            //     T1[time_log_counter] = Measures.lidar_beg_time;
            //     s_plot[time_log_counter] = t5 - t0;
            //     s_plot2[time_log_counter] = feats_undistort->points.size();
            //     s_plot3[time_log_counter] = kdtree_incremental_time;
            //     s_plot4[time_log_counter] = kdtree_search_time;
            //     s_plot5[time_log_counter] = kdtree_delete_counter;
            //     s_plot6[time_log_counter] = kdtree_delete_time;
            //     s_plot7[time_log_counter] = kdtree_size_st;
            //     s_plot8[time_log_counter] = kdtree_size_end;
            //     s_plot9[time_log_counter] = aver_time_consu;
            //     s_plot10[time_log_counter] = add_point_size;
            //     time_log_counter ++;
            //     printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
            //     ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            //     fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
            //     <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
            //     dump_lio_state_to_log(fp);
            // }
            return true;
        } else
        {
            return false;
        }

        // status = ros::ok();
        // rate.sleep();
    // }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    // if (pcl_wait_save->size() > 0 && pcd_save_en)
    // {
    //     string file_name = string("scans.pcd");
    //     string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    //     pcl::PCDWriter pcd_writer;
    //     cout << "current scan saved to /PCD/" << file_name<<endl;
    //     pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    // }

    // fout_out.close();
    // fout_pre.close();

    // if (runtime_pos_log)
    // {
    //     vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
    //     FILE *fp2;
    //     string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    //     fp2 = fopen(log_dir.c_str(),"w");
    //     fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
    //     for (int i = 0;i<time_log_counter; i++){
    //         fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
    //         t.push_back(T1[i]);
    //         s_vec.push_back(s_plot9[i]);
    //         s_vec2.push_back(s_plot3[i] + s_plot6[i]);
    //         s_vec3.push_back(s_plot4[i]);
    //         s_vec5.push_back(s_plot[i]);
    //     }
    //     fclose(fp2);
    // }

    return false;
}
