#include "sliding_window.hpp"
#include <iostream>
#include <numeric>

using namespace std;
using namespace Eigen;

// 构造函数，初始化窗口大小
SlidingWindow::SlidingWindow(size_t window_size) : window_size_(window_size) {}

// 向窗口添加新数据
void SlidingWindow::push_back(double value)
{
    // 如果窗口已经满了，移除最旧的数据
    if (window_.size() == window_size_)
    {
        window_.erase(window_.begin());
    }
    // 添加新的数据到窗口
    window_.push_back(value);
}

// 获取当前窗口的均值
double SlidingWindow::get_average() const
{
    if (window_.empty())
    {
        cout << "Window is empty!" << endl;
        return 0.0;
    }
    if (window_.size() < window_size_)
    {
        // 如果窗口未满，返回 0
        return 0.0;
    }
    // 计算窗口内数据的均值
    double sum = accumulate(window_.begin(), window_.end(), 0.0);
    return sum / window_.size();
}

// 获取窗口中超过指定值 num 的元素个数
size_t SlidingWindow::get_count_above(double num) const
{
    if (window_.empty())
    {
        cout << "Window is empty!" << endl;
        return 0;
    }

    // 计算窗口中大于 num 的元素个数
    size_t count = 0;
    for (const auto &value : window_)
    {
        if (value > num)
        {
            count++;
        }
    }
    return count;
}

// 汉宁窗函数（可选）
void SlidingWindow::apply_hanning_window(VectorXd &data_segment) const
{
    for (size_t i = 0; i < data_segment.size(); ++i)
    {
        data_segment(i) *= 0.5 * (1 - cos(2 * M_PI * i / (data_segment.size() - 1)));
    }
}

// 计算窗口内的指定频率范围内的能量
double SlidingWindow::calculate_energy_in_range(double fs, double f1, double f2) const
{
    if (window_.size() < window_size_)
        return 0.0;

    size_t N = window_.size(); // 数据点数
    vector<double> windowed_data(window_); // 复制窗口数据
    VectorXd data_segment = Map<VectorXd>(windowed_data.data(), N);

    // 执行FFT
    fftw_complex* fft_output = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_plan plan = fftw_plan_dft_r2c_1d(N, data_segment.data(), fft_output, FFTW_ESTIMATE);
    fftw_execute(plan);

    // 计算频率分辨率和对应频率范围
    double freq_resolution = fs / N; // 频率分辨率
    size_t f1_index = static_cast<size_t>(f1 / freq_resolution); // 起始频率的索引
    size_t f2_index = static_cast<size_t>(f2 / freq_resolution); // 结束频率的索引

    // 筛选频率范围内的能量
    double energy = 0.0;
    for (size_t i = f1_index; i <= f2_index && i < N / 2; ++i) // 只计算正频率部分
    {
        double real = fft_output[i][0];
        double imag = fft_output[i][1];
        double magnitude = sqrt(real * real + imag * imag); // 幅值
        energy += magnitude * magnitude; // 幅值的平方累加
    }

    // 释放资源
    fftw_destroy_plan(plan);
    fftw_free(fft_output);

    return energy;
}
