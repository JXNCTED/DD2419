#include "odometry/ekf.hpp"

EKF::EKF(const size_t &n,
         const ProcessModelFunc &f,
         const ObservationModelFunc &h,
         const VecMatFunc &j_f)
    : n(n), f(f), h(h), jacobian_f(j_f), x(n)
{
}

void EKF::init(const Eigen::VectorXd &x0) { x = x0; }

auto EKF::predict(const Eigen::VectorXd &u, const double &dt) -> Eigen::VectorXd
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::MatrixXd F = jacobian_f(x);

    x = f(x, u, dt);
    P = F * P * F.transpose() + Q;

    return x;
}

auto EKF::update(const Eigen::VectorXd &z, const VecMatFunc &j_h)
    -> Eigen::VectorXd
{
    std::lock_guard<std::mutex> lock(mtx);

    H = j_h(x);

    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x += K * (z - h(x));
    P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;

    return x;
}
