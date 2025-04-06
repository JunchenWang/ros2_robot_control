#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "robot_math/JointTrajectoryPlanner.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace controllers
{

    class JointMotionController : public controller_interface::ControllerInterface
    {
    public:
        using CmdType = std_msgs::msg::Float64MultiArray;
        JointMotionController() : real_time_buffer_(nullptr)
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            auto js = *real_time_buffer_.readFromRT();
            auto &cmd = command_->get<double>("position");
            auto &cmd_v = command_->get<double>("velocity");
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            command_->get<int>("mode")[0] = 1;
            if (js)
            {
                auto goal = js->data;
                if (!planner.has_same_goal(goal))
                {
                    planner.generate_speed(q, dq, goal, 1.5);
                    
                    last_time_ = node_->now();
                }
                auto dt = node_->now() - last_time_;
                std::vector<double> q, dq, ddq;
                if (planner.evaluate(dt.seconds(), q, dq, ddq))
                {
                    cmd = q;
                    cmd_v = dq;
                }
            }
            else
            {
                cmd = q0_;
                cmd_v = dq0_;
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {

            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            real_time_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
            q0_ = state_->get<double>("position");
            dq0_ = state_->get<double>("velocity");
            robot_math::forward_kinematics(robot_, q0_, T0_);
            command_receiver_ = node_->create_subscription<CmdType>(
                "~/commands",rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg)
                {
                    real_time_buffer_.writeFromNonRT(msg);
                });
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            command_receiver_ = nullptr;
            real_time_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
            return CallbackReturn::SUCCESS;
        }

    protected:
        rclcpp::Subscription<CmdType>::SharedPtr command_receiver_;
        realtime_tools::RealtimeBuffer<CmdType::SharedPtr> real_time_buffer_;
        std::vector<double> q0_;
        std::vector<double> dq0_;
        Eigen::Matrix4d T0_;
        robot_math::JointTrajectoryPlanner planner;
        rclcpp::Time last_time_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::JointMotionController, controller_interface::ControllerInterface)