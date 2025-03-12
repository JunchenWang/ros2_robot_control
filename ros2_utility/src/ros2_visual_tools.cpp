#include "ros2_utility/ros2_visual_tools.hpp"

using namespace Eigen;
using namespace std;

// 构造函数，初始化ROS2节点和所需的发布器
// @param node 共享指针，指向ROS2节点
ROS2VisualTools::ROS2VisualTools(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node)
{
    // 创建一个发布器，用于发布Marker信息，主题名称为"visualization_marker"
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // 创建一个TF广播器，用于发布TF变换
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // 初始化点Marker属性（仅设置一次）
    point_marker_.ns = "end_effector";                            // Marker的命名空间
    point_marker_.type = visualization_msgs::msg::Marker::SPHERE; // Marker的类型，这里使用球体
    point_marker_.color.r = 173.0f / 255.0f;                      // 浅蓝色
    point_marker_.color.g = 216.0f / 255.0f;
    point_marker_.color.b = 230.0f / 255.0f;
    point_marker_.color.a = 1.0; // 不透明

    // 初始化线段属性（仅设置一次）
    line_marker_.ns = "end_effector_path";
    line_marker_.id = 0; // 固定ID，与点Marker区分
    line_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker_.action = visualization_msgs::msg::Marker::ADD;
    line_marker_.color.r = 173.0f / 255.0f; // 与点相同的浅蓝色
    line_marker_.color.g = 216.0f / 255.0f;
    line_marker_.color.b = 230.0f / 255.0f;
    line_marker_.color.a = 1.0; // 不透明
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
    Quaterniond quaternion(rotation_matrix);                       // 将旋转矩阵转换为四元数

    // 设置四元数的旋转信息
    transformStamped.transform.rotation.w = quaternion.w();
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();

    // 广播变换
    tf_broadcaster_->sendTransform(transformStamped);
}

// 发布点Marker，用于Rviz等可视化工具显示位置
// @param position Marker的位置，类型为Vector3d
// @param frame_id Marker所在的坐标系ID
// @param marker_size Marker的大小，默认为1.0
void ROS2VisualTools::publishPointMarker(const Vector3d &position, const std::string &frame_id, double marker_size)
{
    point_marker_.header.frame_id = frame_id;                                       // 设置Marker的坐标系
    point_marker_.header.stamp = rclcpp::Clock().now();                             // 设置当前时间戳
    point_marker_.id = std::chrono::steady_clock::now().time_since_epoch().count(); // 使用时间戳作为ID，确保唯一性

    // 设置Marker的位置
    point_marker_.pose.position.x = position.x();
    point_marker_.pose.position.y = position.y();
    point_marker_.pose.position.z = position.z();
    point_marker_.pose.orientation.w = 1.0; // 设置无旋转（单位四元数）

    // 设置Marker的缩放
    point_marker_.scale.x = 0.005 * marker_size; // x轴缩放
    point_marker_.scale.y = 0.005 * marker_size; // y轴缩放
    point_marker_.scale.z = 0.005 * marker_size; // z轴缩放

    // 发布Marker
    marker_pub_->publish(point_marker_);
}

// 发布线Marker，用于Rviz等可视化工具显示位置
// @param position Marker的位置，类型为Vector3d
// @param frame_id Marker所在的坐标系ID
// @param marker_size Marker的大小，默认为1.0
void ROS2VisualTools::publishLineMarker(const Vector3d &position, const std::string &frame_id, double marker_size)
{
    if (++publish_counter_ % PUBLISH_INTERVAL != 0)
        return; // 跳过本次线段发布
    // 添加当前点到路径列表
    geometry_msgs::msg::Point p;
    p.x = position.x();
    p.y = position.y();
    p.z = position.z();
    path_points_.push_back(p);
    if (path_points_.size() > MAX_POINTS)
        path_points_.erase(path_points_.begin()); // 删除最早的点

    // 更新线段Marker属性
    line_marker_.header.frame_id = frame_id;
    line_marker_.header.stamp = rclcpp::Clock().now(); // 设置当前时间戳
    line_marker_.points = path_points_;                // 设置所有历史点

    // 设置Marker的缩放
    line_marker_.scale.x = 0.003 * marker_size; // 线宽

    // 发布线段
    marker_pub_->publish(line_marker_);
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