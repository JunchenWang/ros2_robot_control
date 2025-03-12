#ifndef MATH_UTILS
#define MATH_UTILS

#include "robot_math/robot_math.hpp"

class MathUtils
{
public:
    // 将输入 dataVector(i) 限幅在 -rangeVector(i) 到 rangeVector(i) 的范围
    template <typename T>
    static void limitVectorWithinRanges(T &dataVector, const T &rangeVector)
    {
        if (dataVector.size() != rangeVector.size())
        {
            throw std::invalid_argument("Input vectors must have the same dimension");
        }
        for (int i = 0; i < dataVector.size(); i++)
        {
            if (rangeVector(i) < 0)
                throw std::invalid_argument("rangeVector must be a positive value");
            dataVector(i) = std::max(-rangeVector(i), std::min(rangeVector(i), dataVector(i)));
        }
        return;
    }
    // 将计算的力矩 tau_d_calculated 限幅在 tau_d_last ± threshold 的范围
    static Eigen::VectorXd saturateTorque(const Eigen::VectorXd &tau_d_calculated,
                                          const Eigen::VectorXd &tau_d_last,
                                          double threshold);
};

#endif // MATH_UTILS
