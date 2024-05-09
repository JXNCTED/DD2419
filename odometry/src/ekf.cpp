#include "odometry/ekf.hpp"

EKF::EKF(const size_t &n,
         const ProcessModelFunc &f,
         const VecMatFunc &j_f,
         const Eigen::MatrixXd &Q,
         const Eigen::MatrixXd &P)
    : n(n), f(f), jacobian_f(j_f), Q(Q), P(P), x(n)
{
    H = Eigen::MatrixXd::Identity(n, n);
    K = Eigen::MatrixXd::Identity(n, n);
}

void EKF::init(const Eigen::VectorXd &x0) { x = x0; }

auto EKF::predict(const Eigen::VectorXd &u, const double &dt) -> Eigen::VectorXd
{
    Eigen::MatrixXd F = jacobian_f(x, dt);

    x = f(x, u, dt);
    P = F * P * F.transpose() + Q;

    return x;
}

auto EKF::update(const Eigen::VectorXd &z,
                 const VecMatFunc &j_h,
                 const Eigen::MatrixXd &R,
                 const double &dt) -> Eigen::VectorXd
{
    H = j_h(x, dt);

    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x = x + K * (z - H * x);
    P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;

    return x;
}
