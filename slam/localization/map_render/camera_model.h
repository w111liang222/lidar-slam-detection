#ifndef __CAMERA_MODEL_H
#define __CAMERA_MODEL_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "slam_base.h"

#define EIGEN_DATA_TYPE_DEFAULT_OPTION Eigen::AutoAlign

template < int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_d = Eigen::Matrix< double, M, 1, option >;

template < int M >
using eigen_vec = eigen_vec_d< M >;

typedef eigen_vec< 3 >                                                  vec_3;
typedef Eigen::Quaternion< double, EIGEN_DATA_TYPE_DEFAULT_OPTION >     eigen_q;

template<typename T>
inline T getSubPixel(cv::Mat &mat, const double &row, const  double &col, double pyramid_layer = 0)
{
	int floor_row = floor(row);
	int floor_col = floor(col);
	double frac_row = row - floor_row;
	double frac_col = col - floor_col;
	int ceil_row = floor_row + 1;
	int ceil_col = floor_col + 1;
    if (pyramid_layer != 0)
    {
        int pos_bias = pow(2, pyramid_layer - 1);
        floor_row -= pos_bias;
        floor_col -= pos_bias;
        ceil_row += pos_bias;
        ceil_row += pos_bias;
    }
    return ((1.0 - frac_row) * (1.0 - frac_col) * (T)mat.ptr<T>(floor_row)[floor_col]) +
           (frac_row * (1.0 - frac_col) * (T)mat.ptr<T>(ceil_row)[floor_col]) +
           ((1.0 - frac_row) * frac_col * (T)mat.ptr<T>(floor_row)[ceil_col]) +
           (frac_row * frac_col * (T)mat.ptr<T>(ceil_row)[ceil_col]);
}

class CameraModel {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraModel(CamParamType param);
    virtual ~CameraModel();
    void setPose(Eigen::Matrix4d &pose);
    Eigen::Matrix4d getPose();
    Eigen::Matrix3d getCameraPara();
    bool project3DPoints(const vec_3 &pt_w, vec_3 &pt_cam, double &u, double &v, const double &scale = 1.0);
    bool point2DAvailable(const int &cols, const int &rows, const double &u, const double &v, const double &scale = 1.0);

  protected:
    Eigen::Matrix3d rot_ext_i2c;
    Eigen::Vector3d pos_ext_i2c;
    eigen_q m_pose_w2c_q = eigen_q::Identity();
    vec_3 m_pose_w2c_t = vec_3(0, 0, 0);
    eigen_q m_pose_c2w_q = eigen_q::Identity();
    vec_3 m_pose_c2w_t = vec_3(0, 0, 0);
    Eigen::Matrix3d m_cam_K;
    double fx, fy, cx, cy;
};

#endif // __CAMERA_MODEL_H