#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/robot_interface.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char **argv)
{
    // To avoid unused parameter warnings
    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<hardware_interface::SensorInterface> loader("hardware_interface", "hardware_interface::SensorInterface");

    try
    {
        std::shared_ptr<hardware_interface::SensorInterface> my_hard = loader.createSharedInstance("hardwares::FTSRSensor");
        my_hard->initialize("sr", "", rclcpp::NodeOptions(), true);
        rclcpp::spin(my_hard->get_node()->get_node_base_interface());
    }
    catch (pluginlib::PluginlibException &ex)
    {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    return 0;
}