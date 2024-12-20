#include <pluginlib/class_loader.hpp>
#include <hardware_interface/robot_interface.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  rclcpp::init(argc, argv);

  pluginlib::ClassLoader<hardware_interface::RobotInterface> loader("hardware_interface", "hardware_interface::RobotInterface");

  try
  {
    std::shared_ptr<hardware_interface::HardwareInterface> my_hard = loader.createSharedInstance("hardwares::SimulationRobot");
    my_hard->initialize("test", "");
    rclcpp::spin(my_hard->get_node()->get_node_base_interface());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }
  
  return 0;
}