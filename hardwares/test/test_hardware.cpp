#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/robot_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
int main(int argc, char **argv)
{
    // To avoid unused parameter warnings
    rclcpp::init(argc, argv);
    auto robot_ = std::make_shared<franka::Robot>("192.168.1.100");
    // pluginlib::ClassLoader<hardware_interface::RobotInterface> loader("hardware_interface", "hardware_interface::RobotInterface");

    // try
    // {
    //     std::shared_ptr<hardware_interface::RobotInterface> my_hard = loader.createSharedInstance("hardwares::FC3Robot");
    //     my_hard->initialize("sr", "", rclcpp::NodeOptions(), true);
    //     rclcpp::spin(my_hard->get_node()->get_node_base_interface());
    // }
    // catch (pluginlib::PluginlibException &ex)
    // {
    //     printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    // }

    return 0;
}