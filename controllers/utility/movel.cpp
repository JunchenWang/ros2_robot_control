#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::vector<double> goal;
    for(int i = 1; i < argc; i++)
      goal.push_back(atof(argv[i]));
    
    auto node = std::make_shared<rclcpp::Node>("movel");
    auto publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("CartesianMotionController/commands", rclcpp::SystemDefaultsQoS());
    std_msgs::msg::Float64MultiArray msg;
    msg.data = goal;
    publisher_->publish(msg);
    rclcpp::shutdown();
    return 0;
}