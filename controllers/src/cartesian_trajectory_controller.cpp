#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "ros2_utility/ros2_visual_tools.hpp"
#include "robot_math/trajectory.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace controllers
{

    class CartesianTrajectoryController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;
        CartesianTrajectoryController() 
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto trajectory = handle_pair.second;
            auto &cmd = command_->get<double>("pose");
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
            }
            command_->get<int>("mode")[0] = 0;
            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    pose0_ = pose;
                }
                else
                {
                    auto goal = std::vector<double>(goal_handle->get_goal()->target_position.data.end() - 6, goal_handle->get_goal()->target_position.data.end());
                    auto goal_T = robot_math::pose_to_tform(goal);
                    auto errs = robot_math::distance(goal_T, T);
                    auto dt = node_->now() - last_time_;
                    if (errs.first < 1e-2 && errs.second < 1e-5 && dt.seconds() >= trajectory->total_time()) // rv and pos
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        pose0_ = pose;
                        // RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                    }
                    else
                    {
                        
                        Eigen::Matrix4d T;
                        Eigen::Vector6d V, dV;
                        trajectory->evaluate(dt.seconds(), T, V, dV);
                        cmd = robot_math::tform_to_pose(T);
                        visual_tools_->publishMarker(T.block(0, 3, 3, 1), "base", 0.5);
                    }
                }
            }
            else
            {
                cmd = pose0_;
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            visual_tools_ = std::make_shared<ROS2VisualTools>(node_);
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            real_time_buffer_.reset();
            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                                      std::shared_ptr<const ACTION::Goal> goal)
            {
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                trajectory->set_traj(goal_handle->get_goal()->target_position.data);
                last_time_ = node_->now();
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});
            };
            // must be member variable
            // otherwise, the callback group will be destroyed before the action server
            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_,
                "~/goal",
                handle_goal,
                handle_cancel,
                handle_accepted,
                rcl_action_server_get_default_options(),
                call_back_group_);
            
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        std::vector<double> q0_;
        std::vector<double> pose0_;
        rclcpp::Time last_time_;
        std::shared_ptr<ROS2VisualTools> visual_tools_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianTrajectoryController, controller_interface::ControllerInterface)