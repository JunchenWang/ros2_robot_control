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
    
    auto node = std::make_shared<rclcpp::Node>("movej");
    auto publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("JointMotionController/commands", rclcpp::SystemDefaultsQoS());
    std_msgs::msg::Float64MultiArray msg;
    msg.data = goal;
    std::this_thread::sleep_for(1s);
    publisher_->publish(msg);
    rclcpp::shutdown();
    return 0;
}