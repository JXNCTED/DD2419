#pragma once
#include <eigen3/Eigen/Dense>
#include <mutex>

class EKF
{
   public:
    using ProcessModelFunc     = Eigen::VectorXd (*)(Eigen::VectorXd,
                                                 Eigen::VectorXd,
                                                 const double &);
    using ObservationModelFunc = Eigen::VectorXd (*)(Eigen::VectorXd);
    using VecMatFunc  = Eigen::MatrixXd (*)(Eigen::VectorXd, const double &);
    using VoidMatFunc = Eigen::MatrixXd (*)();

    EKF() = default;
    EKF(const size_t &n,
        const ProcessModelFunc &f,
        const VecMatFunc &j_f,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &P);

    void init(const Eigen::VectorXd &x0);

    auto predict(const Eigen::VectorXd &u, const double &dt) -> Eigen::VectorXd;

    auto update(const Eigen::VectorXd &z,
                const VecMatFunc &j_h,
                const Eigen::MatrixXd &R,
                const double &dt) -> Eigen::VectorXd;

    auto getState() -> Eigen::VectorXd { return x; }

    auto isInitialized() -> bool { return initialized; }

   private:
    bool initialized = false;
    // system dim
    size_t n;

    // process function
    ProcessModelFunc f;

    // process jacobian
    VecMatFunc jacobian_f;

    // observation jacobian
    Eigen::MatrixXd H;

    // process noise covariance
    Eigen::MatrixXd Q;

    // error state covariance
    Eigen::MatrixXd P;

    // Kalman gain
    Eigen::MatrixXd K;

    Eigen::VectorXd x;
};