#pragma once

#include "Eigen/Dense"

class KalmanFilter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KalmanFilter();
    ~KalmanFilter();

    void Initialization(bool is_static, Eigen::VectorXd &x_in);

    void SetX(Eigen::VectorXd &x_in);

    void Prediction(double dt, Eigen::Matrix<float, 4, 4> &m, float dh);

    bool KFUpdate(Eigen::VectorXd &z);

    Eigen::VectorXd& GetX();

public:
    // heading angle is reversed by 180
    int reverse_count_;

    // object is static
    bool is_static_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transistion matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;
};