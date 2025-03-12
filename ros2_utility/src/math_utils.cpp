#include "ros2_utility/math_utils.hpp"

// 将计算的力矩 tau_d_calculated 限幅在 tau_d_last ± threshold 的范围
Eigen::VectorXd MathUtils::saturateTorque(const Eigen::VectorXd &tau_d_calculated,
                                          const Eigen::VectorXd &tau_d_last,
                                          double threshold)
{
    // 检查输入维度是否一致
    if (tau_d_calculated.size() != tau_d_last.size())
        throw std::invalid_argument("Input vectors must have the same dimension");

    // 检查阈值是否为正数
    if (threshold <= 0)
        throw std::invalid_argument("threshold must be a positive value");

    // 初始化结果向量
    Eigen::VectorXd tau_d_saturated = tau_d_calculated;

    // 对每个分量进行饱和处理
    for (int i = 0; i < tau_d_calculated.size(); i++)
    {
        double delta = tau_d_calculated(i) - tau_d_last(i);

        // 限制变化率
        if (delta > threshold)
            tau_d_saturated(i) = tau_d_last(i) + threshold;
        else if (delta < -threshold)
            tau_d_saturated(i) = tau_d_last(i) - threshold;
    }

    return tau_d_saturated;
}