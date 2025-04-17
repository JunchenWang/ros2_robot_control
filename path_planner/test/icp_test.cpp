#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <pcl/io/pcd_io.h>                    // PCL文件输入输出头文件
#include <pcl/point_types.h>                  // PCL点云数据类型头文件
#include <pcl/registration/icp.h>             // 迭代最近点算法头文件
#include <pcl/visualization/pcl_visualizer.h> // PCL可视化头文件

int main(int argc, char **argv)
{
    // 创建输入点云（源点云）：每行有5个点（横向排列）点云只有1行（点云以无组织形式存储，无明确空间网格关系）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(500, 1));
    // 创建输出点云（目标点云）：空点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // 为输入点云填充随机数据
    for (auto &point : *cloud_in)
    {
        // 生成范围在[0, 1024)的随机坐标
        point.x = 1024 * rand() / (RAND_MAX + 1.0f); // X轴随机坐标
        point.y = 1024 * rand() / (RAND_MAX + 1.0f); // Y轴随机坐标
        point.z = 1024 * rand() / (RAND_MAX + 1.0f); // Z轴随机坐标
    }

    // 打印输入点云信息
    std::cout << "保存了 " << cloud_in->size() << " 个输入数据点:" << std::endl;
    for (auto &point : *cloud_in)
        std::cout << point << std::endl; // 运算符重载，输出点的XYZ坐标

    // 深拷贝输入点云到输出点云（此时两者完全相同）
    *cloud_out = *cloud_in;

    // 修改输出点云数据：所有点的X坐标增加0.7
    std::cout << "点云大小:" << cloud_out->size() << std::endl;
    for (auto &point : *cloud_out)
        point.x += 0.7f; // 在X轴方向施加固定偏移

    // 打印变换后的输出点云
    std::cout << "变换后的 " << cloud_in->size() << " 个数据点:" << std::endl;
    for (auto &point : *cloud_out)
        std::cout << point << std::endl;

    // 创建ICP配准对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);  // 设置源点云（待配准点云）
    icp.setInputTarget(cloud_out); // 设置目标点云（参考点云）

    // 执行配准计算
    pcl::PointCloud<pcl::PointXYZ> Final; // 存储配准结果
    icp.align(Final);                     // 执行对齐计算

    // 输出配准结果
    std::cout << "ICP算法 " << (icp.hasConverged() ? "收敛成功" : "未收敛")
              << ", 配准得分: " << icp.getFitnessScore() << std::endl; // 得分越小越好
    std::cout << "最终变换矩阵:\n"
              << icp.getFinalTransformation() << std::endl; // 4x4变换矩阵

    // 创建可视化窗口
    pcl::visualization::PCLVisualizer viewer("ICP Result - PCL");
    viewer.setBackgroundColor(0.1, 0.1, 0.1); // 背景设为深灰色

    // 添加点云（需转换颜色和尺寸）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        source_color(cloud_in, 255, 0, 0); // 红色：原始输入
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(cloud_out, 0, 255, 0); // 绿色：目标
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        final_color(Final.makeShared(), 0, 0, 255); // 蓝色：配准结果

    viewer.addPointCloud<pcl::PointXYZ>(cloud_in, source_color, "source");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_out, target_color, "target");
    viewer.addPointCloud<pcl::PointXYZ>(Final.makeShared(), final_color, "final");

    // 设置点云显示大小
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "final");

    // 启动交互
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}