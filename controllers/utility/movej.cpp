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
  auto publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("JointMotionController/commands",
                                                                             rclcpp::SystemDefaultsQoS());
  std::ifstream fin("data.txt");
  std::thread t([=, &fin]
                {
                  rclcpp::Rate r(0.1s);
                  std_msgs::msg::Float64MultiArray msg;
                  msg.data.resize(6);
      while(fin >> msg.data[0] >> msg.data[1] >> msg.data[2] >> msg.data[3] >> msg.data[4] >> msg.data[5])
      {
        publisher_->publish(msg);
        r.sleep();
    } });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}