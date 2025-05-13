#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "robot_math/JointTrajectoryPlanner.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_math/TrapezoidFunction.hpp"
namespace controllers
{

    class JointMotionController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        JointMotionController() : speed_(0.5), planner(0.02)
        {
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            auto goal_handle = *real_time_buffer_.readFromRT();
            auto &cmd = command_->get<double>("position");
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            // inital reading should be put here!!
            if(period.seconds() == 0)
            {
                q0_ = q;
            }
            command_->get<int>("mode")[0] = 1;
            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    q0_ = q;
                    planner.reset();
                    //RCLCPP_INFO(node_->get_logger(), "Goal canceled");
                }
                else
                {
                    auto goal = goal_handle->get_goal()->target_position.data;
                    
                    double err = robot_math::distance(q, goal);
                    if (err < 1e-5)
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        q0_ = q;
                        planner.reset();
                        //RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                    }
                    else
                    {
                        int flag = 0;
                        if (!planner.is_valid() || !planner.has_same_goal(goal))
                        {
                            //RCLCPP_INFO(node_->get_logger(), "%f %f", goal[0], dq[0]);
                            planner.generate_speed(q, dq, goal, speed_);
                            flag = 1;
                            last_time_ = node_->now();
                        }
                        auto dt = node_->now() - last_time_;
                        std::vector<double> interp_q, interp_dq, interp_ddq;
                        planner.evaluate(dt.seconds(), interp_q, interp_dq, interp_ddq);
                        cmd = interp_q;
                        // if(period.seconds() > 0)
                        // {
                        //     double diff = q[0] - interp_q[0];
                        //     if(std::abs(diff) > 0.01)
                        //     {
                        //         planner.generate_speed(q, dq, goal, speed_);
                        //         RCLCPP_INFO(node_->get_logger(), "attention %d %f %f", flag, dt.seconds(), speed_);
                        //         RCLCPP_INFO(node_->get_logger(), "q: %f %f %f %f %f %f", q[0], q[1], q[2], q[3], q[4], q[5]);
                        //         RCLCPP_INFO(node_->get_logger(), "interp_q: %f %f %f %f %f %f", interp_q[0], interp_q[1], interp_q[2], interp_q[3], interp_q[4], interp_q[5]);
                        //         RCLCPP_INFO(node_->get_logger(), "goal: %f %f %f %f %f %f", goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
                        //         RCLCPP_INFO(node_->get_logger(), "dq: %f %f %f %f %f %f", dq[0], dq[1], dq[2], dq[3], dq[4], dq[5]);
                        //         planner.evaluate(0, interp_q, interp_dq, interp_ddq);
                        //         RCLCPP_INFO(node_->get_logger(), "interp_q with 0: %f %f %f %f %f %f", interp_q[0], interp_q[1], interp_q[2], interp_q[3], interp_q[4], interp_q[5]);
                        //         throw std::runtime_error("goal not reached");
                        //     }
                        // }
                        // RCLCPP_INFO(node_->get_logger(), "%f %f", q[0], dq[0]);
                        // auto feedback = std::make_shared<ACTION::Feedback>();
                        // feedback->current_position.data = q;
                        // goal_handle->publish_feedback(feedback);
                    }
                }
            }
            else
            {
                cmd = q0_;
            }
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {

            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            node_->get_parameter_or<double>("speed", speed_, 0.5);
            real_time_buffer_.reset();
            planner.reset();

            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                                   std::shared_ptr<const ACTION::Goal> goal)
            {
                //RCLCPP_INFO(node_->get_logger(), "Received goal request");
                (void)uuid;
                // auto goal_handle = *real_time_buffer_.readFromNonRT();
                // if(goal_handle)
                // {
                //     RCLCPP_INFO(node_->get_logger(), "Goal already in progress");
                //     return rclcpp_action::GoalResponse::REJECT;
                // }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                //RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                // this needs to return quickly to avoid blocking the executor,
                // so we declare a lambda function to be called inside a new thread
                // auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
                // std::thread{execute_in_thread}.detach();
                // RCLCPP_INFO(node_->get_logger(), "accept goal");
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
        // rclcpp::Subscription<CmdType>::SharedPtr command_receiver_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<GoalHandle>> real_time_buffer_;
        std::vector<double> q0_;
        robot_math::JointTrajectoryPlanner<robot_math::TrapezoidFunction> planner;
        rclcpp::Time last_time_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        double speed_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::JointMotionController, controller_interface::ControllerInterface)