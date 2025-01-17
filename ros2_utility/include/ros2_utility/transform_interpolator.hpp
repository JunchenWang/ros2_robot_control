#ifndef TRANSFORM_INTERPOLATOR
#define TRANSFORM_INTERPOLATOR

/**
 * @class TransformInterpolator
 * @brief 该类用于动态完成两个变换矩阵之间的插值计算，最大速度为10mm/s
 *
 * 该类的目标是计算一个齐次变换矩阵向量vector<Matrix4d> &T_traj的平滑插值。通过设定插值的速度比例 k (0~1) 可以控制插值的快慢
   示例详见 transform_interpolator_test.cpp
 */

#include "robot_math/robot_math.hpp"

using namespace std;
using namespace Eigen;
using namespace robot_math;

class TransformInterpolator
{
public:
    // 构造函数
    TransformInterpolator(const vector<Matrix4d> &T_traj, double k = 0.1, double dt = 0.002, bool print_detail = false);

    // 追加一个变换矩阵
    void addTransform(const Matrix4d &T);

    // 追加正弦轨迹，生成在xoy平面，并支持正弦波大小比例调节
    void appendSineTrajectory(int total_sine_points, double amplitude, double period);

    // 设置速度比例 (0 ~ 1)
    void setSpeedFactor(double k);

    // 是否已经完成插值
    bool isInterpolationComplete() const;

    // 执行一次插值计算，返回当前插值的变换矩阵
    Matrix4d step();

    // 执行一次插值计算，同时设置速度比例
    Matrix4d step(double k);

    // 设置下一个轨迹段
    bool setNextTransform();

    // 返回当前轨迹段的索引
    size_t getCurrentIndex() const;

    // S型轨迹插值4个点
    void append4Transform(const Matrix4d &Tdip);

    // 将 T_traj_ 的后 num 个齐次变换矩阵乘以 Tdip 倒序追加到 T_traj_ 末尾
    void appendNumTransform(int num, const Matrix4d &Tdip);

// private:
    vector<Matrix4d> T_traj_; // 轨迹点集合
    double k_;                // 当前速度比例
    double dt_;               // 时间步长
    bool print_detail_;       // 是否打印详细信息
    size_t current_index_;    // 当前轨迹段的索引
    double t_;                // 当前插值因子
    double distance_;         // 起始到目标的距离
    Vector6d V_;              // T1 到 T2 的对数映射
};

#endif // TRANSFORM_INTERPOLATOR
