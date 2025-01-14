#ifndef SLIDINGWINDOW
#define SLIDINGWINDOW

#include <vector>
#include "robot_math/robot_math.hpp"
#include <fftw3.h>

using namespace std;
using namespace Eigen;

class SlidingWindow {
public:
    // 构造函数，初始化窗口大小
    SlidingWindow(size_t window_size);

    // 向窗口添加新数据
    void push_back(double value);

    // 获取当前窗口的均值
    double get_average() const;
    
    // 获取窗口中超过指定值 num 的元素个数
    size_t get_count_above(double num) const;

    // 汉宁窗
    void apply_hanning_window(Eigen::VectorXd &data_segment) const;

    // 计算窗口内的指定频率范围内的能量
    double calculate_energy_in_range(double fs, double f1, double f2) const;

private:
    size_t window_size_;          // 窗口大小
    vector<double> window_;  // 存储窗口数据
};

#endif // SLIDINGWINDOW
