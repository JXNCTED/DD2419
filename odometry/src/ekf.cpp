#include "odometry/ekf.hpp"

EKF::EKF(const size_t &n,
         const ProcessModelFunc &f,
         const ObservationModelFunc &h,
         const VecMatFunc &j_f,
         const VecMatFunc &j_h,
         const VoidMatFunc &update_q,
         const VecMatFunc &update_r,
         const Eigen::MatrixXd &P0)
    : n(n), f(f), h(h), jacobian_f(j_f), jacobian_h(j_h), update_Q(update_q), update_R(update_r), P_post(P0), x_pri(n), x_post(n)
{
}

void EKF::init(const Eigen::VectorXd &x0) { x_post = x0; }

Eigen::VectorXd EKF::predict(const Eigen::VectorXd &u)
{
    F = jacobian_f(x_post);
    Q = update_Q();

    x_pri = f(x_post, u);
    P_pri = F * P_post * F.transpose() + Q;

    x_post = x_pri;
    P_post = P_pri;
    return x_pri;
}

Eigen::VectorXd EKF::update(const Eigen::VectorXd &z)
{
    H                       = jacobian_h(x_pri);
    R                       = update_R(z);
    K                       = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    x_post                  = x_pri + K * (z - h(x_pri));
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    P_post                  = (I - K * H) * P_pri;

    return x_post;
}