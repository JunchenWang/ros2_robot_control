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
        ForwardController()
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            auto js = *real_time_buffer_.readFromRT();
            auto &cmd_interface = command_->get<double>(cmd_name_);
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(robot_, q, T);
            auto pose = robot_math::tform_to_pose(T);
            // inital reading should be put here!!
            if(period.seconds() == 0)
            {
                q0_ = q;
                pose0_ = pose;
                dq0_ = dq;
            }
            command_->get<int>("mode")[0] = mode_;
            if (js)
            {
                cmd_interface = js->data;
            }
            else
            {
                if (mode_ == 0)
                    cmd_interface = pose0_;
                else if (mode_ == 1)
                    cmd_interface = q0_;
                else if (mode_ == 2)
                    cmd_interface = dq0_;
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            node_->get_parameter_or<std::string>("cmd_name", cmd_name_, "");
            if (cmd_name_ == "pose")
                mode_ = 0;
            else if (cmd_name_ == "position")
                mode_ = 1;
            else if (cmd_name_ == "velocity")
                mode_ = 2;
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "cmd_name %s is not supported!", cmd_name_.c_str());
                return CallbackReturn::FAILURE;
            }
            real_time_buffer_.reset();
            command_receiver_ = node_->create_subscription<CmdType>(
                "~/commands", rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg)
                {
                    real_time_buffer_.writeFromNonRT(msg);
                });
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            command_receiver_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        rclcpp::Subscription<CmdType>::SharedPtr command_receiver_;
        realtime_tools::RealtimeBuffer<CmdType::SharedPtr> real_time_buffer_;
        std::string cmd_name_;
        int mode_;
        std::vector<double> q0_;
        std::vector<double> dq0_;
        std::vector<double> pose0_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForwardController, controller_interface::ControllerInterface)