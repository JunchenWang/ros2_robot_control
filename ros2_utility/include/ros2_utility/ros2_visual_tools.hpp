#ifndef ROS2_VISUAL_TOOLS_HPP
#define ROS2_VISUAL_TOOLS_HPP

#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include "robot_math/robot_math.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// ROS2VisualTools类，封装了TF广播和Marker发布功能
class ROS2VisualTools
{
public:
    // 构造函数，初始化ROS2节点和发布器
    ROS2VisualTools(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);

    // 广播TF变换，更新坐标系之间的转换关系
    // @param transform_matrix 4×4齐次变换矩阵
    // @param frame_id 父坐标系的ID
    // @param child_frame_id 子坐标系的ID
    void broadcastTransform(
        const Eigen::Matrix4d &transform_matrix,
        const std::string &frame_id,
        const std::string &child_frame_id);
    // 发布Marker，用于可视化
    // @param position Marker的位置，Eigen::Vector3d 格式
    // @param frame_id Marker所在的坐标系ID
    // @param marker_size Marker的大小，默认为1.0
    void publishMarker(const Eigen::Vector3d &position, const std::string &frame_id, double marker_size = 1.0);

    // 保存任意长度的向量数据到文件
    void saveToFile(const Eigen::VectorXd &data, const std::string &file_path);

private:
    // 发布器，用于发送Marker消息到Rviz等可视化工具
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // TF广播器，用于广播TF变换
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // ROS2_VISUAL_TOOLS_HPP
