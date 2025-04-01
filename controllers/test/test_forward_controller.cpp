#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_forward_controller");
    auto publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("JointMotionController/commands", rclcpp::SystemDefaultsQoS());
    rclcpp::Time start = node->now();
    int cnt = 1;
    auto timer_callback =
      [=,&cnt]() -> void {
        std_msgs::msg::Float64MultiArray msg;
        double t = (node->now() - start).seconds();
        double y = std::sin( t);
        if(cnt & 1)
            msg.data = {3.1415926, 0, 0, 0, 0, 0};
        else 
            msg.data = {0, 0, 0, 0, 0, 0};
        msg.data = {y, 0, 0, 0, 0, 0};
        cnt++;
        publisher_->publish(msg);
      };
    auto timer_ = node->create_wall_timer(0.02s, timer_callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}