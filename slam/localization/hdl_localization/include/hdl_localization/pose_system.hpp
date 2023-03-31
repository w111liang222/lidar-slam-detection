#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z,
                  gyro_x, gyro_y, gyro_z]
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;
public:
  PoseSystem() {
    dt = 0.01;
    imu_extrinic = Quaterniont::Identity();
  }

  // system equation (without input)
  VectorXt f(const VectorXt& state) const {
    VectorXt next_state(23);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);
    Vector3t gyro = state.middleRows(16, 3);
    Quaterniont dq_ext(state[19], state[20], state[21], state[22]);
    dq_ext.normalize();

    // position
    next_state.middleRows(0, 3) = pt + vt * dt;  //

    // velocity
    next_state.middleRows(3, 3) = vt;

    // orientation
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();

    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity
    next_state.middleRows(16, 3) = state.middleRows(16, 3);  // constant angular velocity
    next_state.middleRows(19, 4) << dq_ext.w(), dq_ext.x(), dq_ext.y(), dq_ext.z();
    return next_state;
  }

  // system equation
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(23);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);
    Quaterniont dq_ext(state[19], state[20], state[21], state[22]);
    dq_ext.normalize();

    Vector3t raw_acc = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);

    // position
    next_state.middleRows(0, 3) = pt + vt * dt;  //

    // velocity
    Vector3t g(0.0f, 0.0f, 9.81f);
    Vector3t acc_ = dq_ext * (imu_extrinic.inverse() * (raw_acc - acc_bias));
    Vector3t acc = qt * acc_;
    next_state.middleRows(3, 3) = vt + (acc - g) * dt;
    // next_state.middleRows(3, 3) = vt; // + (acc - g) * dt;		// acceleration didn't contribute to accuracy due to large noise

    // orientation
    Vector3t gyro = raw_gyro - gyro_bias;
    gyro = dq_ext * (imu_extrinic.inverse() * gyro);
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity
    next_state.middleRows(16, 3) = gyro;
    next_state.middleRows(19, 4) << dq_ext.w(), dq_ext.x(), dq_ext.y(), dq_ext.z();

    return next_state;
  }

  // observation equation
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
  Quaterniont imu_extrinic;
};

}

#endif // POSE_SYSTEM_HPP
