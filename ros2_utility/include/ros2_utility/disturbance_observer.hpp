#ifndef DISTURBANCE_OBSERVER
#define DISTURBANCE_OBSERVER

#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/math_utils.hpp"

/**
 * @class DisturbanceObserver
 * @brief 扰动观测器类，用于计算力矩扰动并更新观测状态。
 *
 * @param Y 观测矩阵，必须为方阵
 * @param limit 力矩扰动的限制向量，维度必须与Y的行、列数一致
 * @param filter_on 是否开启滤波器，默认为true
 * @param filter_size 滤波器窗口大小，默认为50
 *
 * @example
 * @code
 * // 初始化示例
 * Eigen::MatrixXd Y = Eigen::MatrixXd::Identity(7, 7);
 * Eigen::VectorXd limit{10, 10, 10, 10, 5, 5, 5};
 * DisturbanceObserver observer(Y, limit, true, 100);
 * // 计算力矩扰动
 * Eigen::VectorXd torque_d = observer.computeTorqueDisturbance(J, dq, tau_calc, M, dt);
 * @endcode
 */

class DisturbanceObserver
{
public:
    DisturbanceObserver(const Eigen::MatrixXd &Y,
                        const Eigen::VectorXd &limit,
                        bool filter_on = true,
                        int filter_size = 50)
        : Y_(Y), Z_(Eigen::VectorXd::Zero(Y.cols())), limit_(limit), filter_on_(filter_on), filter_(Y.cols(), filter_size) {}

    // 计算力矩扰动并更新观测状态，注意 tau_calc 表示计算得到的力矩向量，即 tau_task + tau_null，不包含 C 和 G
    Eigen::VectorXd computeTorqueDisturbance(const Eigen::MatrixXd &J,
                                             const Eigen::VectorXd &dq,
                                             const Eigen::VectorXd &tau_calc,
                                             const Eigen::LDLT<Eigen::MatrixXd> &M_ldlt,
                                             double dt);

private:
    Eigen::MatrixXd Y_;
    Eigen::VectorXd Z_;
    Eigen::VectorXd limit_;
    bool filter_on_;
    robot_math::MovingFilter<double> filter_;
};

#endif // DISTURBANCE_OBSERVER