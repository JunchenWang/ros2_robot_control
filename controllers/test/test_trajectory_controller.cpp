#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <fstream>
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("movej");
  auto publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("CartesianTrajectoryController/commands",
                                                                             rclcpp::SystemDefaultsQoS());
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
      0, -0.5, -.1, .1, 3.14, 0, 0,
      1, -0.5, 0, .2, 3.14, 0, 0,
      2, -0.5, .1, .2, 3.14, 0, 0,
      3, -0.5, .2, .1, 3.14, 0, 0,
      4, -0.5, .1, 0, 3.14, 0, 0,
      5, -0.5, 0, 0, 3.14, 0, 0};
  std::this_thread::sleep_for(1s);
  publisher_->publish(msg);
  rclcpp::shutdown();
  return 0;
}