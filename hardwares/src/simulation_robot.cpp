#include "hardware_interface/robot_interface.hpp"
#include <iostream>
#include <vector>

namespace hardwares
{
    class SimulationRobot : public hardware_interface::RobotInterface
    {
    public:
        SimulationRobot()
        {
        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::read(t, period);
            state_["position"] = {1, 2, 3, 4, 5, 6};
            state_["velocity"] = {1, 2, 3, 4, 5, 6};
            //std::dynamic_pointer_cast<hardware_interface::FTSensorInterface>(ft_sensor_)->compensate_gravity({0, 0, 0, 0, 0, 0});
            auto &force = loaned_state_["ft_sensor"]->at("force");
            std::cerr << "force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            state_["position"] = command_["position"];
            state_["velocity"] = command_["velocity"];
            state_["torque"] = command_["torque"];
        }
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationRobot, hardware_interface::RobotInterface)