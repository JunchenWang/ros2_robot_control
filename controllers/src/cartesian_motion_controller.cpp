#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "robot_math/CartesianTrajectoryPlanner.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace controllers
{

    class CartesianMotionController : public controller_interface::ControllerInterface
    {
    public:
        using CmdType = std_msgs::msg::Float64MultiArray;
        CartesianMotionController() : real_time_buffer_(nullptr)
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            auto js = *real_time_buffer_.readFromRT();
            auto &cmd = command_->get<double>("pose");
            auto &q = state_->get<double>("position");
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(robot_, q, T);
            auto &dq = state_->get<double>("velocity");
            command_->get<int>("mode")[0] = 0;
            if (js)
            {
                auto goal = js->data;
                if (!planner.has_same_goal(goal))
                {
                    auto Te = robot_math::pose_to_tform(goal);
                    planner.generate_speed(T, Te, 0.5);
                    last_time_ = node_->now();
                }
                auto dt = node_->now() - last_time_;
                Eigen::Matrix4d T;
                Eigen::Vector6d V, dV;
                if (planner.evaluate(dt.seconds(), T, V, dV))
                {
                    cmd = robot_math::tform_to_pose(T);
                }
            }
            else
            {
                cmd = pose0_;
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            real_time_buffer_.reset();
            q0_ = state_->get<double>("position");
            dq0_ = state_->get<double>("velocity");
            robot_math::forward_kinematics(robot_, q0_, T0_);
            pose0_ = robot_math::tform_to_pose(T0_);
            planner.generate(T0_, T0_, 1);
            //RCLCPP_INFO(node_->get_logger(), "%f %f %f %f %f %f", pose0_[0], pose0_[1],pose0_[2],pose0_[3],pose0_[4],pose0_[5]);
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
        std::vector<double> q0_;
        std::vector<double> dq0_;
        std::vector<double> pose0_;
        Eigen::Matrix4d T0_;
        robot_math::CartesianTrajectoryPlanner planner;
        rclcpp::Time last_time_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianMotionController, controller_interface::ControllerInterface)