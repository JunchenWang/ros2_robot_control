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
  auto node = std::make_shared<rclcpp::Node>("test_joint_controller");
  auto client = rclcpp_action::create_client<ACTION>(node, "JointMotionController/goal");

  if (!client->wait_for_action_server())
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return 1;
  }

  // auto send_goal_options = rclcpp_action::Client<ACTION>::SendGoalOptions();
  // send_goal_options.goal_response_callback = [node](const GoalHandle::SharedPtr &goal_handle)
  // {
  //   if (!goal_handle)
  //   {
  //     RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
  //   }
  //   else
  //   {
  //     RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
  //   }
  // };

  // send_goal_options.feedback_callback = [node](
  //                                           GoalHandle::SharedPtr,
  //                                           const std::shared_ptr<const ACTION::Feedback> feedback)
  // {
  //   std::stringstream ss;
  //   for (auto number : feedback->current_position.data)
  //   {
  //     ss << number << " ";
  //   }
  //   RCLCPP_INFO(node->get_logger(), ss.str().c_str());
  // };

  // send_goal_options.result_callback = [node](const GoalHandle::WrappedResult &result)
  // {
  //   switch (result.code)
  //   {
  //   case rclcpp_action::ResultCode::SUCCEEDED:
  //     break;
  //   case rclcpp_action::ResultCode::ABORTED:
  //     RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
  //     return;
  //   case rclcpp_action::ResultCode::CANCELED:
  //     RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
  //     return;
  //   default:
  //     RCLCPP_ERROR(node->get_logger(), "Unknown result code");
  //     return;
  //   }
  //   std::stringstream ss;
  //   ss << "Result received: " << result.result->success << " ";
  //   RCLCPP_INFO(node->get_logger(), ss.str().c_str());
  // };
  auto goal_msg = ACTION::Goal();
  goal_msg.target_position.data = {0, 0, 0, 0, 0, 0};
  auto handle_future = client->async_send_goal(goal_msg);
  auto result = rclcpp::spin_until_future_complete(node, handle_future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    rclcpp::shutdown();
    return 0;
  }
  auto handle = handle_future.get();
  if(handle == nullptr)
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
  result_future.get().result->success;
  rclcpp::shutdown();
  return 0;
}