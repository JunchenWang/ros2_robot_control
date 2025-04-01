#include "robot_hardware_interface/robot_interface.hpp"
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
            // auto &force = com_state_["ft_sensor"]->get<double>("force");
            // std::cerr << "raw force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            auto &cmd = command_.get<double>();
            auto &state = state_.get<double>();
            auto mode = command_.get<int>("mode")[0];
            if (mode == 0) // cartisan space
            {
                auto it = cmd.find("pose");
                if (it != cmd.end())
                {
                    Eigen::Map<Eigen::Matrix4d> T(&it->second[0]);
                    auto jt = inverse_kinematics(state["position"], T);
                    for (int i = 0; i < dof_; i++)
                        state["velocity"][i] = (jt[i] - state["position"][i]) / period.seconds();
                    state["position"] = jt;
                }
            }
            else if (mode == 1) // joint space
            {
                for (int i = 0; i < dof_; i++)
                    state["velocity"][i] = (cmd["position"][i] - state["position"][i]) / period.seconds();
                state["position"] = cmd["position"];
                //state["velocity"] = cmd["velocity"];
            }
            else if (mode == 2) // velocity
            {
                state["velocity"] = cmd["velocity"];
                for (int i = 0; i < dof_; i++)
                    state["position"][i] += cmd["velocity"][i] * period.seconds();
            }
        }
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationRobot, hardware_interface::RobotInterface)