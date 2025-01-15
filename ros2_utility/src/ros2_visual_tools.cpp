#include "ros2_utility/ros2_visual_tools.hpp"

using namespace Eigen;
using namespace std;

// 构造函数，初始化ROS2节点和所需的发布器
// @param node 共享指针，指向ROS2节点
ROS2VisualTools::ROS2VisualTools(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
{
    // 创建一个发布器，用于发布Marker信息，主题名称为"visualization_marker"
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // 创建一个TF广播器，用于发布TF变换
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

// 广播TF变换，更新坐标系之间的转换关系
// @param transform_matrix 4×4齐次变换矩阵
// @param frame_id 父坐标系的ID
// @param child_frame_id 子坐标系的ID
void ROS2VisualTools::broadcastTransform(
    const Matrix4d &transform_matrix, // 输入4×4齐次变换矩阵
    const std::string &frame_id,
    const std::string &child_frame_id)
{
    // 创建一个TransformStamped消息，包含时间戳、父坐标系和子坐标系信息
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = rclcpp::Clock().now(); // 使用当前时间戳
    transformStamped.header.frame_id = frame_id;           // 父坐标系ID
    transformStamped.child_frame_id = child_frame_id;      // 子坐标系ID

    // 从齐次变换矩阵中提取平移信息（前三列第四行）
    transformStamped.transform.translation.x = transform_matrix(0, 3);
    transformStamped.transform.translation.y = transform_matrix(1, 3);
    transformStamped.transform.translation.z = transform_matrix(2, 3);

    // 从齐次变换矩阵中提取旋转信息（左上角3×3矩阵），并转换为四元数
    Matrix3d rotation_matrix = transform_matrix.block<3, 3>(0, 0); // 提取旋转矩阵
    Quaterniond quaternion(rotation_matrix); // 将旋转矩阵转换为四元数

    // 设置四元数的旋转信息
    transformStamped.transform.rotation.w = quaternion.w();
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();

    // 广播变换
    tf_broadcaster_->sendTransform(transformStamped);
}


// 发布Marker，用于Rviz等可视化工具显示位置
// @param position Marker的位置，类型为Vector3d
// @param frame_id Marker所在的坐标系ID
// @param marker_size Marker的大小，默认为1.0
void ROS2VisualTools::publishMarker(const Vector3d &position, const std::string &frame_id, double marker_size)
{
    // 创建一个Marker消息，定义可视化对象的属性
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;                   // 设置Marker的坐标系
    marker.header.stamp = rclcpp::Clock().now();         // 设置当前时间戳
    marker.ns = "end_effector";                           // Marker的命名空间
    marker.id = std::chrono::steady_clock::now().time_since_epoch().count();  // 使用时间戳作为ID，确保唯一性
    marker.type = visualization_msgs::msg::Marker::SPHERE; // Marker的类型，这里使用球体

    // 设置Marker的位置
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0; // 设置无旋转（单位四元数）

    // 设置Marker的缩放
    marker.scale.x = 0.005 * marker_size; // x轴缩放
    marker.scale.y = 0.005 * marker_size; // y轴缩放
    marker.scale.z = 0.005 * marker_size; // z轴缩放

    // 设置Marker的颜色 浅蓝色
    marker.color.r = 173.0f / 255.0f;  // 红色通道，范围 [0, 1]
    marker.color.g = 216.0f / 255.0f;  // 绿色通道，范围 [0, 1]
    marker.color.b = 230.0f / 255.0f;  // 蓝色通道，范围 [0, 1]
    marker.color.a = 1.0;  // 透明度，1为不透明

    // 发布Marker
    marker_pub_->publish(marker);
}

void ROS2VisualTools::saveToFile(const VectorXd &data, const std::string &file_path)
{
    // 打开文件流
    std::ofstream fout(file_path, std::ios::app); // 使用追加模式
    if (!fout.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2VisualTools"), "Failed to open file: %s", file_path.c_str());
        return;
    }

    // 遍历向量元素，按空格分隔写入
    for (int i = 0; i < data.size(); ++i)
    {
        fout << data(i);
        if (i != data.size() - 1)
        {
            fout << " "; // 元素间空格分隔
        }
    }
    fout << std::endl; // 换行

    // 关闭文件流
    fout.close();
}