#include "transform_interpolator.hpp"
#include <iostream>

// 构造函数，接收轨迹点集合
TransformInterpolator::TransformInterpolator(const vector<Matrix4d> &T_traj, double k, double dt, bool print_detail)
    : T_traj_(T_traj),
      k_(clamp(k, 0.0, 1.0)),                                           // 确保 k 合法 默认速度比例因子 k 初始化为 0.1（可调）
      dt_(dt > 0 ? dt : throw invalid_argument("dt must be positive")), // 设置插值时间步长 确保 dt 为正数
      print_detail_(print_detail),                                      // 是否打印详细信息
      current_index_(0),                                                // 初始化当前轨迹段的索引
      t_(0.0)                                                           // 插值因子 t 初始为 0.0（表示起点）
{
    if (T_traj_.size() < 2)
        throw invalid_argument("Trajectory must contain at least two transforms.");
    // 初始化当前段的距离和对数映射
    distance_ = (T_traj_[current_index_].block(0, 3, 3, 1) - T_traj_[current_index_ + 1].block(0, 3, 3, 1)).norm();
    V_ = logT(T_traj_[current_index_].inverse() * T_traj_[current_index_ + 1]);
}

// 追加一个变换矩阵
void TransformInterpolator::addTransform(const Matrix4d &T)
{
    T_traj_.push_back(T);
    // 如果当前轨迹段为最后一段，更新距离和对数映射
    if (current_index_ == T_traj_.size() - 2)
    {
        distance_ = (T_traj_[current_index_].block(0, 3, 3, 1) - T_traj_[current_index_ + 1].block(0, 3, 3, 1)).norm();
        V_ = logT(T_traj_[current_index_].inverse() * T_traj_[current_index_ + 1]);
    }
}

// 追加正弦轨迹，生成在xoy平面，并支持正弦波大小比例调节及自定义周期
void TransformInterpolator::appendSineTrajectory(int total_sine_points, double amplitude, double period)
{
    // 基于最后一个点生成正弦轨迹
    Matrix4d last_transform = T_traj_.back();
    double last_x = last_transform(0, 3); // 获取最后一个变换矩阵的x坐标
    double last_y = last_transform(1, 3); // 获取最后一个变换矩阵的y坐标
    double last_z = last_transform(2, 3); // 获取最后一个变换矩阵的z坐标
    Matrix3d last_R = last_transform.block(0, 0, 3, 3); // 获取最后一个变换矩阵的旋转矩阵
    double step = period / (total_sine_points - 1); // 步长，x从0到period
    for (int i = 1; i < total_sine_points; ++i)
    {
        double x = last_x + i * step; // x轴从0到period
        double y = last_y + amplitude * sin(2 * M_PI * i * step / period); // y坐标根据自定义周期生成正弦波
        double z = last_z; // z坐标保持不变

        // 创建齐次变换矩阵
        Matrix4d T = Matrix4d::Identity();
        T.block(0, 0, 3, 3) = last_R; // 设置旋转矩阵
        T(0, 3) = x; // 设置x轴位移
        T(1, 3) = y; // 设置y轴位移，应用正弦波
        T(2, 3) = z; // 设置z轴位移，保持不变

        T_traj_.push_back(T);
    }
}

// 设置速度比例 (0 ~ 1)
void TransformInterpolator::setSpeedFactor(double k)
{
    k_ = max(0.0, min(k, 1.0)); // 限制 k 在 [0, 1]
}

// 是否已经完成插值
bool TransformInterpolator::isInterpolationComplete() const
{
    if (current_index_ >= T_traj_.size() - 1 && print_detail_)
        cout << "All " << T_traj_.size() << " trajectories completed ~" << endl;
    return current_index_ >= T_traj_.size() - 1;
}

// 执行一次插值计算，返回当前插值的变换矩阵
Matrix4d TransformInterpolator::step()
{
    if (isInterpolationComplete())
    {
        cout << "Interpolation already completed!" << endl;
        return T_traj_.back(); // 返回最终目标变换矩阵
    }

    // 更新插值因子
    t_ = min(t_ + dt_ * (1.0 / distance_) * k_ / 100, 1.0); // 这里/100是为了调整最大速度为10mm/s

    // 如果 t_ >= 1.0，循环推进轨迹段
    while (t_ >= 1.0)
    {
        double t_remainder = t_ - 1.0; // 计算剩余插值因子
        if (!setNextTransform())      // 切换到下一个轨迹段
        {
            return T_traj_.back();    // 如果所有轨迹完成，返回最后一个目标变换
        }
        t_ = t_remainder;             // 将剩余因子应用到新轨迹段
    }

    // 计算插值变换矩阵
    return T_traj_[current_index_] * exp_twist(V_ * t_);
}

// 执行一次插值计算，同时设置速度比例
Matrix4d TransformInterpolator::step(double k)
{
    setSpeedFactor(k);
    return step();
}

// 设置下一个轨迹段
bool TransformInterpolator::setNextTransform()
{
    if (isInterpolationComplete())
        return false;

    current_index_++; // 切换到下一段轨迹
    if (print_detail_)
        cout << "Trajectory " << current_index_ << " completed." << endl;

    // 计算新轨迹段的距离和对数映射
    distance_ = (T_traj_[current_index_].block(0, 3, 3, 1) - T_traj_[current_index_ + 1].block(0, 3, 3, 1)).norm();
    V_ = logT(T_traj_[current_index_].inverse() * T_traj_[current_index_ + 1]);
    return true;
}

// 返回当前轨迹段的索引
size_t TransformInterpolator::getCurrentIndex() const
{
    return current_index_;
}

// S型轨迹插值4个点
void TransformInterpolator::append4Transform(const Matrix4d &Tdip)
{
    if (T_traj_.size() < 2)
    {
        throw invalid_argument("Trajectory must contain at least two transforms.");
    }
    Matrix4d T1 = T_traj_[T_traj_.size() - 2];
    Matrix4d T2 = T_traj_[T_traj_.size() - 1];
    T_traj_.push_back(T2 * Tdip);
    T_traj_.push_back(T1 * Tdip);
    T_traj_.push_back(T1 * Tdip * Tdip);
    T_traj_.push_back(T2 * Tdip * Tdip);
}

// 将 T_traj_ 的后 num 个齐次变换矩阵乘以 Tdip 倒序追加到 T_traj_ 末尾
void TransformInterpolator::appendNumTransform(int num, const Matrix4d &Tdip)
{
    if (num <= 0 || num > T_traj_.size())
    {
        throw invalid_argument("Invalid num value: it must be between 1 and the size of T_traj_.");
    }
    // 记录当前 T_traj_ 的初始大小
    size_t original_size = T_traj_.size();
    // 从末尾开始倒序取 num 个变换矩阵
    for (int i = 0; i < num; ++i)
    {
        // 倒序索引
        size_t index = original_size - 1 - i;
        // 获取对应的变换矩阵
        const Matrix4d &T = T_traj_[index];
        // 将结果追加到 T_traj_ 末尾
        T_traj_.push_back(T * Tdip);
    }
}
