#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
namespace controllers
{

    class RobotStateBroadcaster : public controller_interface::ControllerInterface
    {
    public:
        RobotStateBroadcaster()
        {
        }
        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            robot_control_msgs::msg::RobotState msg;
            msg.header.stamp = t;
            std::fill_n(std::back_inserter(msg.robot_state), 28, 0);
            std::copy(q.begin(), q.end(), msg.robot_state.begin());
            std::copy(dq.begin(), dq.end(), msg.robot_state.begin() + 7);
            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = msg;
                real_time_publisher_->unlockAndPublish();
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {

            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            real_time_publisher_ = nullptr;
            robot_state_publisher_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::RobotStateBroadcaster, controller_interface::ControllerInterface)