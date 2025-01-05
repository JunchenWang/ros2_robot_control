#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char **argv)
{
    // To avoid unused parameter warnings
    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader("controller_interface", "controller_interface::ControllerInterface");
    std::shared_ptr<controller_interface::ControllerInterface> my_hard;
    try
    {
        my_hard = loader.createSharedInstance("controllers::ForwardController");
        my_hard->initialize("controller", "");
        rclcpp::spin(my_hard->get_node()->get_node_base_interface());
    }
    catch (pluginlib::PluginlibException &ex)
    {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }
    return 0;
}