#include "controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
namespace controllers
{
    class ForwardController : public controller_interface::ControllerInterface
    {
    public:
        ForwardController() : is_new_cmd_available_(false)
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            if (is_new_cmd_available_)
            {
                auto js = real_time_buffer_.readFromRT();
                command_->at("position") = (*js)->position;

                is_new_cmd_available_ = false;
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/) override
        {
            //node_->get_parameter_or<std::string>("cmd_name", joint_command_topic_name_, "gui/joint_state");
            command_receiver_ = node_->create_subscription<sensor_msgs::msg::JointState>("~/joint_cmd", rclcpp::SensorDataQoS(),
                                                                                         std::bind(&ForwardController::robot_joint_command_callback, this, std::placeholders::_1));
            return CallbackReturn::SUCCESS;
        }

    protected:
        void robot_joint_command_callback(sensor_msgs::msg::JointState::SharedPtr js)
        {
            real_time_buffer_.writeFromNonRT(js);
            is_new_cmd_available_ = true;
        }
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_receiver_;
        realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> real_time_buffer_;
        volatile bool is_new_cmd_available_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForwardController, controller_interface::ControllerInterface)