#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace controllers
{
    
    class ForwardController : public controller_interface::ControllerInterface
    {
    public:
        using CmdType = std_msgs::msg::Float64MultiArray;
        ForwardController() : real_time_buffer_(nullptr)
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            auto js = *real_time_buffer_.readFromRT();
            if(js)
            {
                auto &cmd_interface = command_->get<double>(cmd_name_);
                if (js->data.size() != cmd_interface.size())
                {
                    RCLCPP_ERROR(node_->get_logger(), "Size of command does not match size of joint state");
                    return;
                }
                for (size_t i = 0; i < js->data.size(); ++i)
                {
                    cmd_interface[i] = js->data[i];
                }
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/) override
        {
            node_->get_parameter_or<std::string>("cmd_name", cmd_name_, "");
            command_receiver_ = node_->create_subscription<CmdType>(
                "~/commands", rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg) { real_time_buffer_.writeFromNonRT(msg); });
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State &/*previous_state*/) override
        {
            real_time_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/) override
        {
            real_time_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
            return CallbackReturn::SUCCESS;
        }

    protected:
        rclcpp::Subscription<CmdType>::SharedPtr command_receiver_;
        realtime_tools::RealtimeBuffer<CmdType::SharedPtr> real_time_buffer_;
        std::string cmd_name_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForwardController, controller_interface::ControllerInterface)