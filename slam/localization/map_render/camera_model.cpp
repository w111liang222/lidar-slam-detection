#include "camera_model.h"

CameraModel::CameraModel(CamParamType param) {
    m_pose_w2c_q.setIdentity();
    m_pose_w2c_t.setZero();

    Eigen::Matrix4d ext_i2c = param.staticTrans.inverse();
    rot_ext_i2c = ext_i2c.topLeftCorner<3, 3>();
    pos_ext_i2c = ext_i2c.topRightCorner<3, 1>();

    fx = param.K.at<float>(0, 0);
    fy = param.K.at<float>(1, 1);
    cx = param.K.at<float>(0, 2);
    cy = param.K.at<float>(1, 2);

    m_cam_K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
}
CameraModel::~CameraModel() {}

void CameraModel::setPose(Eigen::Matrix4d &pose) {
    m_pose_w2c_t = pose.topLeftCorner<3, 3>() * pos_ext_i2c + pose.topRightCorner<3, 1>();
    m_pose_w2c_q = eigen_q(pose.topLeftCorner<3, 3>() * rot_ext_i2c);

    m_pose_c2w_q = m_pose_w2c_q.inverse();
    m_pose_c2w_t = -(m_pose_w2c_q.inverse() * m_pose_w2c_t);
}

Eigen::Matrix4d CameraModel::getPose() {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.topRightCorner<3, 1>() = m_pose_w2c_t;
    pose.topLeftCorner<3, 3>() = m_pose_w2c_q.toRotationMatrix();
    return pose;
}

Eigen::Matrix3d CameraModel::getCameraPara() {
    return m_cam_K;
}

bool CameraModel::project3DPoints(const vec_3 &pt_w, vec_3 &pt_cam, double &u, double &v, const double &scale) {
    pt_cam = (m_pose_c2w_q * pt_w + m_pose_c2w_t);
    if (pt_cam(2) < 0.001) {
        return false;
    }
    u = (pt_cam(0) * fx / pt_cam(2) + cx) * scale;
    v = (pt_cam(1) * fy / pt_cam(2) + cy) * scale;
    return true;
}

bool CameraModel::point2DAvailable(const int &cols, const int &rows, const double &u, const double &v, const double &scale) {
    const double MARGIN = 0.005;
    if ((u / scale >= (MARGIN * cols + 1)) && (std::ceil(u / scale) < ((1 - MARGIN) * cols)) &&
        (v / scale >= (MARGIN * rows + 1)) && (std::ceil(v / scale) < ((1 - MARGIN) * rows))) {
        return true;
    } else {
        return false;
    }
}