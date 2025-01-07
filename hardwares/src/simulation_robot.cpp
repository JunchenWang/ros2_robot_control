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

        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            auto it = command_.find("position");
            if(it != command_.end())
            {
                state_["position"] = it->second;
                it = command_.find("velocity");
                if(it != command_.end())
                {
                    state_["velocity"] = it->second;
                }
            }
            else
            {
                it = command_.find("velocity");
                if(it != command_.end())
                {
                    auto &p = state_["position"];
                    for(int i = 0; i < dof_; i++)
                       p[i] += it->second[i] * period.seconds();
                    state_["velocity"] = it->second;
                }
            }
            // for(auto & state : state_names_)
            // {
            //     auto it = command_.find(state);
            //     if(it != command_.end())
            //     {
            //         state_[state] = it->second;
            //     }
            // }
            // state_["position"] = command_["position"];
            // state_["velocity"] = command_["velocity"];
        }
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationRobot, hardware_interface::RobotInterface)