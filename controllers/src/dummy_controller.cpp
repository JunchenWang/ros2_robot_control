#include "controller_interface/controller_interface.hpp"
#include <iostream>
namespace controllers
{
    class DummyController : public controller_interface::ControllerInterface
    {
    public:
        DummyController() 
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & /*period*/) override
        {
            command_->get<double>("position") = {std::sin(t.seconds()), 0, 0, 0, 0, 0};
            command_->get<double>("velocity") = {std::cos(t.seconds()), 0, 0, 0, 0, 0};
        }
       
    protected:
      
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::DummyController, controller_interface::ControllerInterface)