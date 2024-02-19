#pragma once
#include <eigen3/Eigen/Dense>

using VecVecFunc  = Eigen::VectorXd (*)(Eigen::VectorXd);
using VecMatFunc  = Eigen::MatrixXd (*)(Eigen::VectorXd);
using VoidMatFunc = Eigen::MatrixXd (*)(void);

class EKF
{
   public:
    EKF() = delete;
    EKF(const size_t &n,
        const VecVecFunc &f,
        const VecVecFunc &h,
        const VecMatFunc &j_f,
        const VecMatFunc &j_h,
        const VoidMatFunc &update_q,
        const VecMatFunc &update_r,
        const Eigen::MatrixXd &P0);

    void init(const Eigen::VectorXd &x0);

    Eigen::VectorXd predict();

    Eigen::VectorXd update(const Eigen::VectorXd &z);

   private:
    // system dim
    const size_t n;

    // process function
    VecVecFunc f;
    // observation function
    VecVecFunc h;

    // process jacobian
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;

    // observation jacobian
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;

    // process noise covariance
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;

    // observe noise covariance
    VecMatFunc update_R;
    Eigen::MatrixXd R;

    // error state covariance
    Eigen::MatrixXd P_pri;
    Eigen::MatrixXd P_post;

    // Kalman gain
    Eigen::MatrixXd K;

    Eigen::VectorXd x_pri;
    Eigen::VectorXd x_post;
};