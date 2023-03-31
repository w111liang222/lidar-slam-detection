#ifndef __TRANSFORM__H
#define __TRANSFORM__H

#include <Eigen/Geometry>

typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Quaterniond Rotation;
typedef Eigen::Vector3d Translation;
typedef Eigen::Matrix<double, 4, 4> Matrix;

class Transform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transform() {
    rotation_.setIdentity();
    translation_.setZero();
  }

  Transform(const Translation& translation, const Rotation& rotation)
      : translation_(translation), rotation_(rotation) {}

  Transform(Matrix matrix)
      : translation_(matrix.topRightCorner<3, 1>()),
        rotation_(matrix.topLeftCorner<3, 3>()) {}

  Transform& operator=(const Transform& trans) {
    rotation_.x() = trans.rotation().x();
    rotation_.y() = trans.rotation().y();
    rotation_.z() = trans.rotation().z();
    rotation_.w() = trans.rotation().w();
    translation_(0) = trans.translation()(0);
    translation_(1) = trans.translation()(1);
    translation_(2) = trans.translation()(2);
    return *this;
  }

  Transform(const Transform& trans) {
    rotation_.x() = trans.rotation().x();
    rotation_.y() = trans.rotation().y();
    rotation_.z() = trans.rotation().z();
    rotation_.w() = trans.rotation().w();
    translation_(0) = trans.translation()(0);
    translation_(1) = trans.translation()(1);
    translation_(2) = trans.translation()(2);
  }

  Transform(const Transform&& trans) {
    rotation_.x() = trans.rotation().x();
    rotation_.y() = trans.rotation().y();
    rotation_.z() = trans.rotation().z();
    rotation_.w() = trans.rotation().w();
    translation_(0) = trans.translation()(0);
    translation_(1) = trans.translation()(1);
    translation_(2) = trans.translation()(2);
  }

  const Rotation& rotation() const { return rotation_; }

  const Translation& translation() const { return translation_; }

  Matrix matrix() const {
    Matrix matrix;
    matrix.setIdentity();
    matrix.topLeftCorner<3, 3>() = rotation_.matrix();
    matrix.topRightCorner<3, 1>() = translation_;
    return matrix;
  }

  Transform inverse() const {
    const Rotation rotation_inverted(rotation_.w(), -rotation_.x(),
                                     -rotation_.y(), -rotation_.z());
    return Transform(-(rotation_inverted * translation_), rotation_inverted);
  }

  Transform operator*(const Transform& rhs) const {
    return Transform(translation_ + rotation_ * rhs.translation(),
                     rotation_ * rhs.rotation());
  }

  static Transform exp(const Vector6& vector) {
    constexpr double kEpsilon = 1e-8;
    const double norm = vector.tail<3>().norm();
    if (norm < kEpsilon) {
      return Transform(vector.head<3>(), Rotation::Identity());
    } else {
      return Transform(vector.head<3>(), Rotation(Eigen::AngleAxisd(
                                             norm, vector.tail<3>() / norm)));
    }
  }

  Vector6 log() const {
    Eigen::AngleAxisd angle_axis(rotation_);
    return (Vector6() << translation_, angle_axis.angle() * angle_axis.axis())
        .finished();
  }

 public:
  Rotation rotation_;
  Translation translation_;
};


static const double Ang2Rad = 0.01745329251994;

// input euler angle unit is deg
inline Transform getTransformFromRPYT(double x, double y, double z, double yaw,
                                      double pitch, double roll) {
  Translation t(x, y, z);
  Eigen::AngleAxisd rollAngle(roll * Ang2Rad, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pitch * Ang2Rad, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(yaw * Ang2Rad, Eigen::Vector3d::UnitZ());
  Rotation r = yawAngle * pitchAngle * rollAngle;
  return Transform(t, r);
}

// output euler angle unit is deg
inline void getRPYTfromTransformFrom(Transform tran, double &x, double &y, double &z,
                                     double &yaw, double &pitch, double &roll) {
  x = tran.translation()(0);
  y = tran.translation()(1);
  z = tran.translation()(2);

  Eigen::Vector3d eulerAngle =
      tran.rotation().toRotationMatrix().eulerAngles(2, 0, 1);
  yaw = eulerAngle(0) / Ang2Rad;
  pitch = eulerAngle(1) / Ang2Rad;
  roll = eulerAngle(2) / Ang2Rad;
}

#endif  //__TRANSFORM__H