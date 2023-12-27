/*
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored,
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored,
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package."
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping."
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <stdio.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib.h"
#include "tools_eigen.hpp"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

template <typename T>
inline void reduce_vector(std::vector<T> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
};

struct Image_frame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using data_type = double;
    using PointType = pcl::PointXYZI;
    int m_if_have_set_intrinsic = 0;
    Eigen::Matrix3d m_cam_K;
    double fx, fy, cx, cy;
    eigen_q m_pose_w2c_q = eigen_q::Identity();
    vec_3 m_pose_w2c_t = vec_3(0, 0, 0);
    eigen_q m_pose_c2w_q = eigen_q::Identity();
    vec_3 m_pose_c2w_t = vec_3(0, 0, 0);
    int m_if_have_set_pose = 0;
    double m_timestamp = 0.0;
    int m_have_solved_pnp = 0;
    eigen_q m_pnp_pose_w2c_q = eigen_q::Identity();
    vec_3 m_pnp_pose_w2c_t = vec_3(0,0,0);

    vec_3 m_pose_t;
    mat_3_3 m_pose_w2c_R;
    int m_img_rows = 0;
    int m_img_cols = 0;
    int m_frame_idx = 0;
    Eigen::Matrix<double, 2, 1> m_gama_para;
    double m_downsample_step[10] = {1.0, 0.5, 0.25, 1.0/8.0, 1.0/16.0, 1.0/24.0, 1.0/32.0, 1.0/64.0 };

    cv::Mat m_img;
    cv::Mat m_raw_img;
    cv::Mat m_img_gray;

    double m_fov_margin = 0.005;

    Image_frame();
    ~Image_frame();
    void refresh_pose_for_projection();
    void set_pose(const eigen_q & pose_w2c_q, const vec_3 & pose_w2c_t );
    void get_pose(StatesGroup &state);
    int set_frame_idx(int frame_idx);
    void set_intrinsic(Eigen::Matrix3d & camera_K);
    Image_frame(Eigen::Matrix3d &camera_K);
    void init_cubic_interpolation();
    void inverse_pose();
    void release_image();
    bool project_3d_to_2d( const pcl::PointXYZI & in_pt, Eigen::Matrix3d & cam_K, double &u, double &v, const double  & scale = 1.0);
    bool project_3d_to_2d( const vec_3 & pt_w, Eigen::Matrix3d & cam_K, double &u, double &v, const double  & scale = 1.0);
    bool if_2d_points_available(const double &u, const double &v, const double &scale = 1.0, double fov_mar = -1.0);
    vec_3 get_rgb(double &u, double v, int layer = 0, vec_3 *rgb_dx = nullptr, vec_3 *rgb_dy = nullptr);
    double get_grey_color(double & u ,double & v, int layer= 0 );
    bool get_rgb( const double & u,  const double & v, int & r, int & g, int & b  );
    void image_equalize(cv::Mat &img, int amp = 10.0);
    void image_equalize();
    bool project_3d_point_in_this_img(const pcl::PointXYZI & in_pt, double &u, double &v,   pcl::PointXYZRGB * rgb_pt = nullptr, double intrinsic_scale = 1.0);
    bool project_3d_point_in_this_img(const vec_3 & in_pt, double &u, double &v, pcl::PointXYZRGB *rgb_pt = nullptr, double intrinsic_scale = 1.0);
};