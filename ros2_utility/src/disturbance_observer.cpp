#include "ros2_utility/disturbance_observer.hpp"

// 计算力矩扰动并更新观测状态，注意 tau_calc 表示计算得到的力矩向量，即 tau_task + tau_null，不包含 C 和 G
Eigen::VectorXd DisturbanceObserver::computeTorqueDisturbance(const Eigen::MatrixXd &J,
                                                              const Eigen::VectorXd &dq,
                                                              const Eigen::VectorXd &tau_calc,
                                                              const Eigen::LDLT<Eigen::MatrixXd> &M_ldlt,
                                                              double dt)
{
    Eigen::VectorXd P = Y_ * dq;
    Eigen::VectorXd tau_dist = Z_ + P;
    Eigen::MatrixXd project_task = J * (M_ldlt.solve(J.transpose()));
    tau_dist = J.transpose() * (project_task.ldlt().solve(J * M_ldlt.solve(tau_dist)));

    if (filter_on_)
        filter_.filtering(tau_dist.data(), tau_dist.data());

    MathUtils::limitVectorWithinRanges(tau_dist, limit_);
    Eigen::VectorXd torque_command_updated = tau_calc - tau_dist;

    Eigen::VectorXd dZ = Y_ * M_ldlt.solve(-Z_ - P - torque_command_updated);
    Z_ += dZ * dt;

    return tau_dist;
}