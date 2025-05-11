#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
namespace controllers
{

    class ForwardController2 : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        ForwardController2()
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            auto goal_handle = *real_time_buffer_.readFromRT();
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
            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    pose0_ = pose;
                    q0_ = q;
                    dq0_ = dq;
                    // RCLCPP_INFO(node_->get_logger(), "Goal canceled");
                }
                else
                {
                    auto &&goal = goal_handle->get_goal()->target_position.data;

                    if (mode_ == 0)
                    {
                        auto goal_T = robot_math::pose_to_tform(goal);
                        auto errs = robot_math::distance(goal_T, T);
                        if (errs.first < 1e-2 && errs.second < 1e-5) // rv and pos
                        {
                            auto result = std::make_shared<ACTION::Result>();
                            result->success = true;
                            goal_handle->succeed(result);
                            pose0_ = pose;
                        }
                        else
                            cmd_interface = goal;
                    }
                    else if (mode_ == 1)
                    {

                        auto err = robot_math::distance(goal, q);
                        if (err < 1e-5)
                        {
                            auto result = std::make_shared<ACTION::Result>();
                            result->success = true;
                            goal_handle->succeed(result);
                            q0_ = q;
                            // RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                        }
                        else
                            cmd_interface = goal;
                    }
                    else if (mode_ == 2)
                    {

                        auto err = robot_math::distance(goal, dq);
                        if (err < 1e-5)
                        {
                            auto result = std::make_shared<ACTION::Result>();
                            result->success = true;
                            goal_handle->succeed(result);
                            dq0_ = dq;
                            // RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                        }
                        else
                            cmd_interface = goal;
                    }
                }
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
                real_time_buffer_.writeFromNonRT(goal_handle);
            };

            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_,
                "~/goal",
                handle_goal,
                handle_cancel,
                handle_accepted);
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        realtime_tools::RealtimeBuffer<std::shared_ptr<GoalHandle>> real_time_buffer_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        std::string cmd_name_;
        int mode_;
        std::vector<double> q0_;
        std::vector<double> dq0_;
        std::vector<double> pose0_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForwardController2, controller_interface::ControllerInterface)