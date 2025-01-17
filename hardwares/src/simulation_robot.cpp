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
            auto &cmd = command_.get<double>();
            auto &state = state_.get<double>();
            auto it = cmd.find("position");
            if(it != cmd.end())
            {
                state["position"] = it->second;
                it = cmd.find("velocity");
                if(it != cmd.end())
                {
                    state["velocity"] = it->second;
                }
            }
            else
            {
                it = cmd.find("velocity");
                if(it != cmd.end())
                {
                    auto &p = state["position"];
                    for(int i = 0; i < dof_; i++)
                       p[i] += it->second[i] * period.seconds();
                    state["velocity"] = it->second;
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