#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <Eigen/Eigen>

#include "so3_math.h"
#include "tools_eigen.hpp"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

#define USE_ikdtree
#define ESTIMATE_GRAVITY  1
#define ENABLE_CAMERA_OBS 1

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)     // Gravity const in Hong Kong SAR, China
#if ENABLE_CAMERA_OBS
#define DIM_OF_STATES (29) // with vio obs
#else
#define DIM_OF_STATES (18) // For faster speed.
#endif
#define DIM_OF_PROC_N (12) // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (0.0001)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) std::vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

static const Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
static const Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
static const Eigen::Vector3d Zero3d(0, 0, 0);
static const Eigen::Vector3f Zero3f(0, 0, 0);

struct Pose6D
{
    typedef double data_type;
    data_type offset_time;
    data_type rot[9];
    data_type acc[3];
    data_type vel[3];
    data_type pos[3];
    data_type gyr[3];
};

template <typename T = double>
inline Eigen::Matrix<T, 3, 3> vec_to_hat(Eigen::Matrix<T, 3, 1> &omega)
{
    Eigen::Matrix<T, 3, 3> res_mat_33;
    res_mat_33.setZero();
    res_mat_33(0, 1) = -omega(2);
    res_mat_33(1, 0) = omega(2);
    res_mat_33(0, 2) = omega(1);
    res_mat_33(2, 0) = -omega(1);
    res_mat_33(1, 2) = -omega(0);
    res_mat_33(2, 1) = omega(0);
    return res_mat_33;
}

template < typename T = double >
T cot(const T theta)
{
    return 1.0 / std::tan(theta);
}

template < typename T = double >
inline Eigen::Matrix< T, 3, 3 > right_jacobian_of_rotion_matrix(const Eigen::Matrix< T, 3, 1 > & omega)
{
    //Barfoot, Timothy D, State estimation for robotics. Page 232-237
    Eigen::Matrix< T, 3, 3>   res_mat_33;

    T theta = omega.norm();
    if(std::isnan(theta) || theta == 0)
        return Eigen::Matrix< T, 3, 3>::Identity();
    Eigen::Matrix< T, 3, 1 > a = omega/ theta;
    Eigen::Matrix< T, 3, 3 > hat_a = vec_to_hat(a);
    res_mat_33 = sin(theta)/theta * Eigen::Matrix< T, 3, 3 >::Identity()
                    + (1 - (sin(theta)/theta))*a*a.transpose()
                    + ((1 - cos(theta))/theta)*hat_a;
    return res_mat_33;
}

template < typename T = double >
Eigen::Matrix< T, 3, 3 > inverse_right_jacobian_of_rotion_matrix(const Eigen::Matrix< T, 3, 1> & omega)
{
    //Barfoot, Timothy D, State estimation for robotics. Page 232-237
    Eigen::Matrix< T, 3, 3>   res_mat_33;

    T theta = omega.norm();
    if(std::isnan(theta) || theta == 0)
        return Eigen::Matrix< T, 3, 3>::Identity();
    Eigen::Matrix< T, 3, 1 > a = omega/ theta;
    Eigen::Matrix< T, 3, 3 > hat_a = vec_to_hat(a);
    res_mat_33 = (theta / 2) * (cot(theta / 2)) * Eigen::Matrix<T, 3, 3>::Identity()
                + (1 - (theta / 2) * (cot(theta / 2))) * a * a.transpose()
                + (theta / 2) * hat_a;
    return res_mat_33;
}

struct StatesGroup
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d rot_end;                                 // [0-2] the estimated attitude (rotation matrix) at the end lidar point
    Eigen::Vector3d pos_end;                                 // [3-5] the estimated position at the end lidar point (world frame)
    Eigen::Vector3d vel_end;                                 // [6-8] the estimated velocity at the end lidar point (world frame)
    Eigen::Vector3d bias_g;                                  // [9-11] gyroscope bias
    Eigen::Vector3d bias_a;                                  // [12-14] accelerator bias
    Eigen::Vector3d gravity;                                 // [15-17] the estimated gravity acceleration

    Eigen::Matrix3d rot_ext_i2c;                             // [18-20] Extrinsic between IMU frame to Camera frame on rotation.
    Eigen::Vector3d pos_ext_i2c;                             // [21-23] Extrinsic between IMU frame to Camera frame on position.
    double          td_ext_i2c_delta;                        // [24]    Extrinsic between IMU frame to Camera frame on position.
    vec_4           cam_intrinsic;                           // [25-28] Intrinsice of camera [fx, fy, cx, cy]
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
    double last_update_time = 0;
    double          td_ext_i2c;
    StatesGroup()
    {
        rot_end = Eigen::Matrix3d::Identity();
        pos_end = vec_3::Zero();
        vel_end = vec_3::Zero();
        bias_g = vec_3::Zero();
        bias_a = vec_3::Zero();
        gravity = Eigen::Vector3d(0.0, 0.0, 9.805);

        //Ext camera w.r.t. IMU
        rot_ext_i2c = Eigen::Matrix3d::Identity();
        pos_ext_i2c = vec_3::Zero();

        cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
        // cov.block(18, 18, 6,6) *= 0.1;
        last_update_time = 0;
        td_ext_i2c_delta = 0;
        td_ext_i2c = 0;
    }

    ~StatesGroup(){}

    StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        StatesGroup a = *this;
        // a.rot_end = this->rot_end * Sophus::SO3d::exp(vec_3(state_add(0, 0), state_add(1, 0), state_add(2, 0) ) );
        a.rot_end = this->rot_end * Exp(state_add(0), state_add(1), state_add(2));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
#endif

        a.cov = this->cov;
        a.last_update_time = this->last_update_time;
#if ENABLE_CAMERA_OBS
        //Ext camera w.r.t. IMU
        a.rot_ext_i2c = this->rot_ext_i2c * Exp(  state_add(18), state_add(19), state_add(20) );
        a.pos_ext_i2c = this->pos_ext_i2c + state_add.block<3,1>( 21, 0 );
        a.td_ext_i2c_delta = this->td_ext_i2c_delta + state_add(24);
        a.cam_intrinsic = this->cam_intrinsic + state_add.block(25, 0, 4, 1);
#endif
        return a;
    }

    StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->vel_end += state_add.block<3, 1>(6, 0);
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        this->gravity += state_add.block<3, 1>(15, 0);
#endif
#if ENABLE_CAMERA_OBS
        //Ext camera w.r.t. IMU
        this->rot_ext_i2c = this->rot_ext_i2c * Exp(  state_add(18), state_add(19), state_add(20));
        this->pos_ext_i2c = this->pos_ext_i2c + state_add.block<3,1>( 21, 0 );
        this->td_ext_i2c_delta = this->td_ext_i2c_delta + state_add(24);
        this->cam_intrinsic = this->cam_intrinsic + state_add.block(25, 0, 4, 1);
#endif
        return *this;
    }

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> a;
        Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = SO3_LOG(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(15, 0) = this->gravity - b.gravity;

#if ENABLE_CAMERA_OBS
        //Ext camera w.r.t. IMU
        Eigen::Matrix3d rotd_ext_i2c(b.rot_ext_i2c.transpose() * this->rot_ext_i2c);
        a.block<3, 1>(18, 0) = SO3_LOG(rotd_ext_i2c);
        a.block<3, 1>(21, 0) = this->pos_ext_i2c - b.pos_ext_i2c;
        a(24) = this->td_ext_i2c_delta - b.td_ext_i2c_delta;
        a.block<4, 1>(25, 0) = this->cam_intrinsic - b.cam_intrinsic;
#endif
        return a;
    }
};

template <typename T>
T rad2deg(T radians)
{
    return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees)
{
    return degrees * PI_M / 180.0;
}

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

#endif
