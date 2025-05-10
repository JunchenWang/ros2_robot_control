#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std::chrono_literals;

using ACTION = robot_control_msgs::action::RobotMotion;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_controller");
  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("ForwardController/commands", 10);
  auto start = node->now();
  rclcpp::WallRate rate(500);
  while (rclcpp::ok())
  {
    auto t = (node->now() - start).seconds();
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {std::sin(0.5 * t), 0, 0, 0, 0, 0};
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}