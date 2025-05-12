#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_msgs/srv/control_command.hpp"
using namespace std::chrono_literals;

using ACTION = robot_control_msgs::action::RobotMotion;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_controller");
  auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>("control_node/control_command");
  if(!controller_client->wait_for_service(1s))
  {
    RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
    rclcpp::shutdown();
    return 1;
  }
  auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
  request->cmd_name = "load";
  request->cmd_params = "controllers::ForwardController2";
  auto future = controller_client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node, future);
  if (result != rclcpp::FutureReturnCode::SUCCESS || !future.get()->result)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load controller");
    rclcpp::shutdown();
    return 1;
  }

  // auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("ForwardController/commands", 10);
  // auto start = node->now();
  // rclcpp::WallRate rate(500);
  // while (rclcpp::ok())
  // {
  //   auto t = (node->now() - start).seconds();
  //   std_msgs::msg::Float64MultiArray msg;
  //   msg.data = {std::sin(2*t), 0, 0, 0, 0, 0};
  //   publisher->publish(msg);
  //   rclcpp::spin_some(node);
  //   rate.sleep();
  // }
  rclcpp::shutdown();
  return 0;
}