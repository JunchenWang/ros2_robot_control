#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "robot_math/CartesianTrajectoryPlanner.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_math/TrapezoidFunction.hpp"
#include "robot_math/LinearFunction.hpp"
namespace controllers
{

    class CartesianMotionController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;

        CartesianMotionController() :  speed_(0.5), planner(0.02)
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {

            auto goal_handle = *real_time_buffer_.readFromRT();
            auto &cmd = command_->get<double>("pose");
            auto &q = state_->get<double>("position");
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(robot_, q, T);
            auto pose = robot_math::tform_to_pose(T);
            auto &dq = state_->get<double>("velocity");
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
                    planner.reset();
                    //RCLCPP_INFO(node_->get_logger(), "Goal canceled");
                }
                else
                {
                    auto goal = goal_handle->get_goal()->target_position.data;
                    auto goal_T = robot_math::pose_to_tform(goal);
                    auto errs = robot_math::distance(goal_T, T);
                    if (errs.first < 1e-4 && errs.second < 1e-5) // rv and pos
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        pose0_ = pose;
                        planner.reset();
                        //RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                    }
                    else
                    {
                        if (!planner.is_valid() || !planner.has_same_goal(goal))
                        {
                            planner.generate_speed(T, goal_T, speed_);

                            last_time_ = node_->now();
                        }
                        auto dt = node_->now() - last_time_;
                        Eigen::Matrix4d T;
                        Eigen::Vector6d V, dV;
                        planner.evaluate(dt.seconds(), T, V, dV);
                        cmd = robot_math::tform_to_pose(T);
                        // RCLCPP_INFO(node_->get_logger(), "%f", q[0]);
                        // auto feedback = std::make_shared<ACTION::Feedback>();
                        // feedback->current_position.data = q;
                        // goal_handle->publish_feedback(feedback);
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

            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            real_time_buffer_.reset();
            planner.reset();
            node_->get_parameter_or<double>("speed", speed_, 0.5);
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
        realtime_tools::RealtimeBuffer<std::shared_ptr<GoalHandle>> real_time_buffer_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        std::vector<double> q0_;
        std::vector<double> pose0_;
        robot_math::CartesianTrajectoryPlanner<robot_math::LinearFunction> planner;
        rclcpp::Time last_time_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        double speed_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianMotionController, controller_interface::ControllerInterface)