#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "ros2_utility/ros2_visual_tools.hpp"
#include "robot_math/trajectory.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace controllers
{

    class CartesianTrajectoryController : public controller_interface::ControllerInterface
    {
    public:
        using CmdType = std_msgs::msg::Float64MultiArray;
        CartesianTrajectoryController() : real_time_buffer_(nullptr)
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            
            auto &cmd = command_->get<double>("pose");
            auto &q = state_->get<double>("position");
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(robot_, q, T);
            auto &dq = state_->get<double>("velocity");
            command_->get<int>("mode")[0] = 0;
            {
                std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
                if (guard.owns_lock())
                {
                    if (new_arrival_)
                    {
                        auto js = *real_time_buffer_.readFromRT();
                        trajectory_.set_traj(js->data);
                        new_arrival_ = false;
                        last_time_ = node_->now();
                    }
                }
            }
            if(trajectory_.is_empty())
            {
                cmd = pose0_;
            }
            else 
            {
                auto dt = node_->now() - last_time_;
                Eigen::Matrix4d T;
                Eigen::Vector6d V, dV;
                trajectory_.evaluate(dt.seconds(), T, V, dV);
                cmd = robot_math::tform_to_pose(T);
                visual_tools_->publishMarker(T.block(0, 3, 3, 1), "base", 0.5);
                
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            visual_tools_ = std::make_shared<ROS2VisualTools>(node_);
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            new_arrival_ = false;
            trajectory_.clear();
            real_time_buffer_.reset();
            q0_ = state_->get<double>("position");
            dq0_ = state_->get<double>("velocity");
            robot_math::forward_kinematics(robot_, q0_, T0_);
            pose0_ = robot_math::tform_to_pose(T0_);
            command_receiver_ = node_->create_subscription<CmdType>(
                "~/commands", rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> guard(mutex_);
                    real_time_buffer_.writeFromNonRT(msg);
                    new_arrival_ = true;
                    
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
        robot_math::CartesianTrajectory trajectory_;
        rclcpp::Time last_time_;
        std::mutex mutex_;
        bool new_arrival_ = false;
        std::shared_ptr<ROS2VisualTools> visual_tools_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianTrajectoryController, controller_interface::ControllerInterface)