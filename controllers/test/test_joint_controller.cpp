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
  auto node = std::make_shared<rclcpp::Node>("test_joint_controller");
  auto client = rclcpp_action::create_client<ACTION>(node, "JointMotionController/goal");
  auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>("control_node/control_command");
  if(!controller_client->wait_for_service(1s))
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

  request->cmd_name = "add";
  request->cmd_params = "RobotStateBroadcaster";
  auto future2 = controller_client->async_send_request(request);
  result = rclcpp::spin_until_future_complete(node, future2);
  if (result != rclcpp::FutureReturnCode::SUCCESS || future2.get()->result != true)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to active controller");
    rclcpp::shutdown();
    return 1;
  }

  if (!client->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return 1;
  }
  
  auto start = node->now();
  rclcpp::WallRate rate(50);
  while (rclcpp::ok())
  {
    auto t = (node->now() - start).seconds();
    auto goal_msg = ACTION::Goal();
    goal_msg.target_position.data = {std::sin(0.5 * t), 0, 0, 0, 0, 0};
    auto handle_future = client->async_send_goal(goal_msg);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}