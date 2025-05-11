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
  std::vector<double> goal;
  for (int i = 1; i < argc; i++)
    goal.push_back(atof(argv[i]));

  auto node = std::make_shared<rclcpp::Node>("movej");
  auto client = rclcpp_action::create_client<ACTION>(node, "JointMotionController/goal");
  auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>("control_node/control_command");
  if (!controller_client->wait_for_service(1s))
  {
    RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
    rclcpp::shutdown();
    return 1;
  }
  auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
  request->cmd_name = "activate";
  request->cmd_params = "JointMotionController";
  auto future = controller_client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node, future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to active controller");
    rclcpp::shutdown();
    return 1;
  }
  if (!client->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting 1s");
    rclcpp::shutdown();
    return 0;
  }

  auto goal_msg = ACTION::Goal();
  goal_msg.target_position.data = goal;
  auto handle_future = client->async_send_goal(goal_msg);
  result = rclcpp::spin_until_future_complete(node, handle_future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    rclcpp::shutdown();
    return 0;
  }
  auto handle = handle_future.get();
  if (handle == nullptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected");
    rclcpp::shutdown();
    return 0;
  }
  auto result_future = client->async_get_result(handle);
  result = rclcpp::spin_until_future_complete(node, result_future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get result");
    rclcpp::shutdown();
    return 0;
  }
  if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(node->get_logger(), "Goal failed");
    rclcpp::shutdown();
    return 0;
  }
  rclcpp::shutdown();
  return 0;
}