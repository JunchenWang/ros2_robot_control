#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/robot_interface.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char **argv)
{
    // To avoid unused parameter warnings
    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<hardware_interface::HardwareInterface> loader("hardware_interface", "hardware_interface::HardwareInterface");

    try
    {
        std::shared_ptr<hardware_interface::HardwareInterface> my_hard = loader.createSharedInstance("hardwares::FTATISensor");
        my_hard->initialize("ati", "", rclcpp::NodeOptions(), true);
        rclcpp::spin(my_hard->get_node()->get_node_base_interface());
    }
    catch (pluginlib::PluginlibException &ex)
    {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    return 0;
}