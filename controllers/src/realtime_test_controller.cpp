#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
namespace controllers
{
    class RealtimeTestController : public controller_interface::ControllerInterface
    {
    public:
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & period) override
        {
            RCLCPP_INFO(node_->get_logger(), "%ld micro sec.", period.nanoseconds() / 1000);
        }
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::RealtimeTestController, controller_interface::ControllerInterface)