#include "controller_interface/controller_interface.hpp"
#include <iostream>
namespace controllers
{
    class Med7Controller : public controller_interface::ControllerInterface
    {
    public:
        Med7Controller() 
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & /*period*/) override
        {
            std::vector<double> &q_cmd_vec = command_->get<double>("position");
            const std::vector<double> &q_vec = state_->get<double>("position");
            q_cmd_vec = q_vec;
        }
       
    protected:
      
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::Med7Controller, controller_interface::ControllerInterface)