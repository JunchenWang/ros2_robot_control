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
                std::vector<double> jt;
                if (inverse_kinematics(state["position"], robot_math::pose_to_tform(cmd["pose"]), jt))
                {
                    if (period.seconds() > 0)
                        for (int i = 0; i < dof_; i++)
                            state["velocity"][i] = (jt[i] - state["position"][i]) / period.seconds();
                    state["position"] = jt;
                }
                else
                    throw std::runtime_error("ik error");
            }
            else if (mode == 1) // joint space
            {
                if (period.seconds() > 0)
                    for (int i = 0; i < dof_; i++)
                        state["velocity"][i] = (cmd["position"][i] - state["position"][i]) / period.seconds();
                state["position"] = cmd["position"];
                // state["velocity"] = cmd["velocity"];
            }
            else if (mode == 2) // velocity
            {
                state["velocity"] = cmd["velocity"];
                for (int i = 0; i < dof_; i++)
                    state["position"][i] += cmd["velocity"][i] * period.seconds();
            }
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
            {
                // to do
                // std::fill(pre_dq_.begin(), pre_dq_.end(), 0);

                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            RobotInterface::on_deactivate(previous_state);
            auto dq = state_.get<double>("velocity");
            std::fill(dq.begin(), dq.end(), 0);
            return CallbackReturn::SUCCESS;
        }
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationRobot, hardware_interface::RobotInterface)