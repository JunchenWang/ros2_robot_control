#include "robot_hardware_interface/sensor_interface.hpp"
#include <iostream>
#include <vector>
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace hardwares
{

    class FTDummySensor : public hardware_interface::SensorInterface
    {
    public:
        FTDummySensor()
        {
        }
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                stop_thread();
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            is_running_ = true;
            thread_ = std::make_unique<std::thread>(
                [this]() -> void
                {
                    rclcpp::WallRate loop_rate(500);
                    while (is_running_)
                    {
                        double t = node_->now().seconds();
                        // geometry_msgs::msg::Wrench::UniquePtr msg = std::make_unique<geometry_msgs::msg::Wrench>();
                        // msg->force.x = std::sin(t);
                        // msg->force.y = std::cos(t);
                        // msg->force.z = 3;
                        // msg->torque.x = 4;
                        // msg->torque.y = 5;
                        // msg->torque.z = 6;
                        //printf("Published message with address: %p\n",reinterpret_cast<std::uintptr_t>(msg.get()));
                        //publisher_->publish(std::move(msg));
                        get<double>("force").writeFromNonRT({std::sin(t), std::cos(t), 3,4,5,6});
                        is_data_comming_ = true;
                        loop_rate.sleep();
                    }
                });
            if(!is_data_comming())
                    return CallbackReturn::FAILURE;

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
        {
            SensorInterface::on_deactivate(previous_state);
            return CallbackReturn::SUCCESS;
        }

    protected:
        //rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::FTDummySensor, hardware_interface::SensorInterface)