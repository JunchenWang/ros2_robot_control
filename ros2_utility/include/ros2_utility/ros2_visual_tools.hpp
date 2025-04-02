#ifndef ROS2_VISUAL_TOOLS_HPP
#define ROS2_VISUAL_TOOLS_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_math/robot_math.hpp"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

// ROS2VisualTools类，封装了TF广播和Marker发布功能
class ROS2VisualTools
{
public:
    // 构造函数，初始化ROS2节点和发布器
    ROS2VisualTools(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node);

    // 广播TF变换，更新坐标系之间的转换关系
    // @param transform_matrix 4×4齐次变换矩阵
    // @param frame_id 父坐标系的ID
    // @param child_frame_id 子坐标系的ID
    void broadcastTransform(
        const Eigen::Matrix4d &transform_matrix,
        const std::string &frame_id,
        const std::string &child_frame_id);
    // 发布点和线Marker，用于可视化
    // @param position Marker的位置
    // @param frame_id Marker所在的坐标系ID
    // @param marker_size Marker的大小，默认为1.0
    void publishPointMarker(const Eigen::Vector3d &position, const std::string &frame_id, double marker_size = 1.0);
    void publishLineMarker(const Eigen::Vector3d &position, const std::string &frame_id, double marker_size = 1.0);

    // STL模型发布函数
    // @param stl_path STL模型的路径
    // @param position 模型的位置
    // @param rotation 模型的旋转矩阵
    // @param frame_id 模型所在的坐标系ID
    // @param color 模型的颜色，默认为白色
    // @param scale 模型的缩放因子，默认为(1, 1, 1)
    void publishSTLMarker(
        const std::string &stl_path,
        const Eigen::Vector3d &position,
        const Eigen::Matrix3d &rotation,
        const std::string &frame_id,
        const std::array<float, 4> &color = {1.0f, 1.0f, 1.0f, 1.0f},
        const Eigen::Vector3d &scale = Eigen::Vector3d(1.0, 1.0, 1.0));

private:
    // 发布器，用于发送Marker消息到Rviz等可视化工具
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    const size_t PUBLISH_INTERVAL = 20;                  // 每PUBLISH_INTERVAL次控制周期发布一次线段
    const size_t MAX_POINTS = 1000;                      // 最大存储点数
    size_t publish_counter_ = 0;                         // 发布计数器
    visualization_msgs::msg::Marker line_marker_;        // 线段Marker对象
    visualization_msgs::msg::Marker point_marker_;       // 点Marker对象
    visualization_msgs::msg::Marker stl_marker_;         // STL模型Marker对象
    std::vector<geometry_msgs::msg::Point> path_points_; // 存储历史点

    // TF广播器，用于广播TF变换
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // ROS2_VISUAL_TOOLS_HPP
