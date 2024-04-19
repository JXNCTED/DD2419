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
    using VecMatFunc           = Eigen::MatrixXd (*)(Eigen::VectorXd);
    using VoidMatFunc          = Eigen::MatrixXd (*)();

    EKF() = delete;
    EKF(const size_t &n,
        const ProcessModelFunc &f,
        const ObservationModelFunc &h,
        const VecMatFunc &j_f);

    void init(const Eigen::VectorXd &x0);

    auto predict(const Eigen::VectorXd &u, const double &dt) -> Eigen::VectorXd;

    auto update(const Eigen::VectorXd &z, const VecMatFunc &j_h)
        -> Eigen::VectorXd;

    auto getState() -> Eigen::VectorXd { return x; }

   private:
    // system dim
    const size_t n;

    // process function
    ProcessModelFunc f;
    // observation function
    ObservationModelFunc h;

    // process jacobian
    VecMatFunc jacobian_f;

    // observation jacobian
    Eigen::MatrixXd H;

    // process noise covariance
    Eigen::MatrixXd Q;

    // observe noise covariance
    Eigen::MatrixXd R;

    // error state covariance
    Eigen::MatrixXd P;

    // Kalman gain
    Eigen::MatrixXd K;

    Eigen::VectorXd x;

    std::mutex mtx;
};