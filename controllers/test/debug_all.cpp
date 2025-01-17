#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_math/OnlineTrajPlanner.h"
#include "control_msgs/msg/vector_data.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include "ros2_utility/sliding_window.hpp"
#include "ros2_utility/transform_interpolator.hpp"
#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace Eigen;
using namespace std;
using namespace robot_math;
using namespace std::chrono;

class DebugAll : public rclcpp::Node
{
public:
    DebugAll() : Node("test_node")
    {
        // test_onlinetrajplanner();
        // test_transforminterpolator1();
        // test_transforminterpolator2();
        // test_slidingwindow();
        // test_fileutils();
        // test_datalogger();
        test_editYAML();
        vec_pub1_ = this->create_publisher<control_msgs::msg::VectorData>("plot1", 10);
        vec_pub2_ = this->create_publisher<control_msgs::msg::VectorData>("plot2", 10);
        test_publisher();
    }

    // 测试 OnlineTrajPlanner 类
    void test_onlinetrajplanner()
    {
        OnlineTrajPlanner planner;
        double vmax[] = {3000, 3000, 3000};
        double acc[] = {1000, 1000, 1000};
        planner.init(2, vmax, acc, 3, 100);
        double foo[] = {1, 1, 1};
        planner.generate(foo);
        rclcpp::Rate rate(500); // 控制循环频率为500Hz
        auto start_time_ = std::chrono::steady_clock::now();
        int count = 0;
        while (rclcpp::ok())
        {
            // 获取当前的轨迹状态
            double pos[3], vel[3], acc_[3];
            if (planner.step(pos, vel, acc_))
            {
                // 输出当前的状态（位置、速度、加速度）
                RCLCPP_INFO(this->get_logger(), "Position: %f %f %f", pos[0], pos[1], pos[2]);

                RCLCPP_INFO(this->get_logger(), "Velocity: %f %f %f", vel[0], vel[1], vel[2]);

                RCLCPP_INFO(this->get_logger(), "Acceleration: %f %f %f", acc_[0], acc_[1], acc_[2]);
                count++;
            }
            else // 轨迹完成
            {
                RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
                break;
            }
            rate.sleep();
        }
        // 轨迹完成后，记录结束时间并计算运行时长
        auto end_time_ = std::chrono::steady_clock::now();
        auto duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - start_time_);

        // 输出总运行时长（毫秒）
        RCLCPP_INFO(this->get_logger(), "Time elapsed: %ld milliseconds", duration_.count());
        RCLCPP_INFO(this->get_logger(), "Total points: %d", count);
    }

    // 测试 TransformInterpolator 类 直线轨迹
    void test_transforminterpolator1()
    {
        // 定义起始变换矩阵 T1 和目标变换矩阵 T2
        Matrix4d T1 = Matrix4d::Identity();
        T1.block(0, 3, 3, 1) = Vector3d(0, 0, 0.001);
        Matrix4d T2 = Matrix4d::Identity();
        T2.block(0, 3, 3, 1) = Vector3d(0, 0.001, 0.001);
        Matrix4d T3 = Matrix4d::Identity();
        T3.block(0, 3, 3, 1) = Vector3d(0.002, 0.001, 0.001);

        vector<Matrix4d> T_traj = {Matrix4d::Identity(), T1, T2, T3};

        // 创建一个 TransformInterpolator 对象并设置速度为 1mm/s
        interpolator_ = std::make_unique<TransformInterpolator>(T_traj, 0.1, 0.002);

        rclcpp::Rate rate(500); // 设置循环频率为500Hz
        int count = 0;
        cout << interpolator_->distance_ << endl;
        // 循环执行插值计算，直到完成
        while (rclcpp::ok() && !interpolator_->isInterpolationComplete())
        {
            // 控制插值速度，前500次使用默认速度，之后降低速度
            if (count < 500)
                Matrix4d T = interpolator_->step(); // 使用当前速度
            else if (count < 1500)
                Matrix4d T = interpolator_->step(0.05); // 降低速度
            else
                Matrix4d T = interpolator_->step(0.2); // 提高速度

            cout << "t: " << interpolator_->t_ << "   ";
            cout << interpolator_->getCurrentIndex() << endl;

            // 增加计数器
            count++;

            // 处理 ROS2 回调
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep(); // 控制循环速率
        }

        // 输出插值完成后的计数结果
        cout << "Interpolation completed in " << count << " steps." << endl;
    }

    // 测试 TransformInterpolator 类 S型轨迹
    void test_transforminterpolator2()
    {
        // 定义起始变换矩阵 T1
        Matrix4d T1 = Matrix4d::Identity();
        vector<Matrix4d> T_traj = {T1, T1};
        interpolator_ = std::make_unique<TransformInterpolator>(T_traj, 0.3, 0.002);
        interpolator_->appendSineTrajectory(1000, 0.01, 0.01);
        Matrix4d Tdip = Matrix4d::Identity();
        Tdip.block(0, 3, 3, 1) = Vector3d(0, 0, 0.001);
        interpolator_->appendNumTransform(1000, Tdip);
        cout << interpolator_->T_traj_.size() << endl;
        rclcpp::Rate rate(500); // 设置循环频率为500Hz
        int count = 0;
        // 循环执行插值计算，直到完成
        while (rclcpp::ok() && !interpolator_->isInterpolationComplete())
        {
            Matrix4d T = interpolator_->step(); // 使用当前速度

            // cout << "t: " << interpolator_->t_ << "   ";
            // cout << interpolator_->getCurrentIndex() << endl;

            // 增加计数器
            count++;

            // 处理 ROS2 回调
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep(); // 控制循环速率
        }
        // 输出插值完成后的计数结果
        cout << "Interpolation completed in " << count << " steps." << endl;
    }

    // 测试 SlidingWindow 类
    void test_slidingwindow()
    {
        // 测试计算时间(ms)
        auto t_start = std::chrono::high_resolution_clock::now();
        // 创建滑动窗口对象
        size_t window_size = 50000; // 窗口大小
        SlidingWindow sliding_window(window_size);

        // 构造测试信号
        double fs = 500;   // 采样频率
        double f1 = 1;     // 第一频率分量
        double f2 = 0.5;   // 第二频率分量
        double amp1 = 1.5; // 第一频率分量幅值
        double amp2 = 0.3; // 第二频率分量幅值

        for (size_t i = 0; i < window_size; ++i)
        {
            double signal = amp1 * sin(2 * M_PI * f1 * i / fs) +
                            amp2 * sin(2 * M_PI * f2 * i / fs);
            sliding_window.push_back(signal);
        }

        // 指定测试频率范围
        double target_f1 = 0; // 测试频率范围下限
        double target_f2 = 2; // 测试频率范围上限

        // 调用函数计算指定频率范围内的幅值和
        double energy = sliding_window.calculate_energy_in_range(fs, target_f1, target_f2);

        cout << "Energy in range [" << target_f1 << ", " << target_f2 << "] Hz: " << energy << endl;
        cout << "Test time(ms): " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t_start).count() * 1000 << endl;
    }

    // 测试 FileUtils 类
    void test_fileutils()
    {
        // 获取ROS2包的共享目录路径
        string package_share_directory = ament_index_cpp::get_package_share_directory("robot_app");
        // 推导出工作空间的根目录
        string workspace_directory = package_share_directory + "/../../../..";
        string log_file_prefix = workspace_directory + "/logs/";
        string filename = log_file_prefix + "Test_2024-12-16 16-07-24 1.2mm切 速度1.txt";
        map<string, vector<double>> dataMap = FileUtils::parseFile(filename);

        vector<double> fd4 = dataMap["fd4"];
        vector<double> fd5 = dataMap["fd5"];
        vector<double> fd6 = dataMap["fd6"];

        vector<double> fd(fd4.size());
        for (size_t i = 0; i < fd4.size(); i++)
            fd[i] = sqrt(fd4[i] * fd4[i] + fd5[i] * fd5[i] + fd6[i] * fd6[i]);

        // for (size_t i = 0; i < fd.size(); i++)
        //     cout << fd[i] << endl;

        SlidingWindow sliding_window(500);

        for (size_t i = 0; i < fd.size(); i++)
        {
            sliding_window.push_back(fd[i]);
            double energy = sliding_window.calculate_energy_in_range(500, 1, 10);
            VectorXd energy_vector(1);
            energy_vector << energy;
            DataLogger::appendToFile(log_file_prefix + "energy.txt", energy_vector);
        }
    }

    // 测试 DataLogger 类
    void test_datalogger()
    {
        // 获取ROS2包的共享目录路径
        string package_share_directory = ament_index_cpp::get_package_share_directory("robot_app");
        // 推导出工作空间的根目录
        string workspace_directory = package_share_directory + "/../../../..";
        string log_file_prefix = workspace_directory + "/logs/";

        double data1[] = {1.1, 1, 1};
        double data2 = 20.2;
        vector<double> data3 = {300.3, 300, 300};
        Vector6d data4 = {4000.4, 4000, 4000, 4000, 4000, 4000};

        double config1[] = {1, 1};
        double config2 = 20;
        vector<double> config3 = {300, 300, 300, 300};
        Vector6d config4 = {4000, 4000, 4000, 4000, 4000, 4000};
        data_logger_ = new DataLogger(
            {
                DATA_WRAPPER_SIZE(data1, 3),
                DATA_WRAPPER(data2),
                DATA_WRAPPER(data3),
                DATA_WRAPPER(data4),
            },
            {
                CONFIG_WRAPPER_SIZE(config1, 2),
                CONFIG_WRAPPER(config2),
                CONFIG_WRAPPER(config3),
                CONFIG_WRAPPER(config4),
            },
            1000);
        rclcpp::Rate rate(500); // 控制循环频率为500Hz
        while (rclcpp::ok())
        {
            for (size_t i = 0; i < 3; i++)
                data1[i] += 1;
            data2 += 1;
            for (size_t i = 0; i < data3.size(); i++)
                data3[i] += 1;
            for (size_t i = 0; i < data4.size(); i++)
                data4[i] += 1;
            if (data2 > 1000)
                break;
            data_logger_->record();

            rclcpp::spin_some(this->get_node_base_interface()); // 处理ROS2回调
            rate.sleep();
        }
        data_logger_->save(log_file_prefix, "debug_all", "测试实验");
        delete data_logger_;
    }

    // 测试发布消息
    void test_publisher()
    {
        auto t_zero = high_resolution_clock::now();
        rclcpp::Rate rate(500); // 控制循环频率为500Hz
        while (rclcpp::ok())
        {
            auto t_start = high_resolution_clock::now();
            auto timestamp = std::chrono::duration<double>(t_start - t_zero).count();
            control_msgs::msg::VectorData msg;
            // 数据分量生成复杂曲线
            // 使用 vector<double> 存储数据
            vector<double> data = {
                sin(2.0 * M_PI * 0.1 * timestamp),      // 正弦曲线，频率为0.1Hz
                cos(2.0 * M_PI * 0.2 * timestamp),      // 余弦曲线，频率为0.2Hz
                sin(2.0 * M_PI * 0.1 * timestamp),      // 正弦曲线，频率为0.1Hz
                cos(2.0 * M_PI * 0.05 * timestamp),     // 余弦曲线，频率为0.05Hz
                sin(2.0 * M_PI * 0.3 * timestamp) * 0.5 // 振幅为0.5的正弦曲线
            };
            msg.data = data;
            msg.timestamp = timestamp;
            msg.name = "data1";
            vec_pub1_->publish(msg);

            // 使用Eigen::Vector6d生成数据
            Eigen::Vector6d data6d = Eigen::Vector6d::Random();
            // 将 Eigen::Vector6d 转换为 std::vector<double>
            vector<double> data_vector(data6d.data(), data6d.data() + data6d.size());
            msg.data = data_vector;
            msg.name = "data2";
            vec_pub2_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface()); // 处理ROS2回调
            rate.sleep();
        }
    }

    // 测试修改 YAML 文件
    void test_editYAML()
    {
        string yaml_file_path = FileUtils::getPackageDirectory("hardwares") + "/config/ur_control.yaml";
        FileUtils::modifyYamlValue(yaml_file_path, "mass", {2.5});
        FileUtils::modifyYamlValue(yaml_file_path, "cog", {3, 1, 2});
        FileUtils::modifyYamlValue(yaml_file_path, "offset", {1.2, 4.6, 6, 8, 9, 12.5});
    }
    unique_ptr<TransformInterpolator> interpolator_;
    DataLogger *data_logger_;
    rclcpp::Publisher<control_msgs::msg::VectorData>::SharedPtr vec_pub1_;
    rclcpp::Publisher<control_msgs::msg::VectorData>::SharedPtr vec_pub2_;
};

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点并运行
    rclcpp::spin(std::make_shared<DebugAll>());

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
