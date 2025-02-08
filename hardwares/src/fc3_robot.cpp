#include "hardware_interface/robot_interface.hpp"
#include "robot_math/robot_math.hpp"
#include <franka/active_torque_control.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <iostream>
#include <vector>

using namespace robot_math;
namespace hardwares
{
    class FC3Robot : public hardware_interface::RobotInterface
    {
    public:
        FC3Robot()
        {
        }
        ~FC3Robot()
        {
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            // RCLCPP_INFO(node_->get_logger(), "%ld micro sec.", period.nanoseconds() / 1000);
            hardware_interface::RobotInterface::write(t, period);
            double dt = 1.0 / update_rate_;
            auto &cmd = command_.get<double>("torque");
            // std::cerr << "torque:" << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << cmd[3] << " "
            //           << cmd[4] << " " << cmd[5] << " " << cmd[6] << std::endl;
            franka::Torques torques{cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]};
            control_->writeOnce(torques);
        }
        bool is_stop() override
        {
            return false;
        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            auto &&state = control_->readOnce().first;
            auto success_rate = state.control_command_success_rate;
            auto && mass = franka_model_->mass(state);
            auto &&coriolis = franka_model_->coriolis(state);
            auto &&gravity = franka_model_->gravity(state);
            auto &&T = state.O_T_EE;
            auto &&tau_d = state.tau_J_d;
            auto &&ext_tau = state.tau_ext_hat_filtered;
            state_.get<double>("success")[0] = success_rate;
            std::copy(mass.begin(), mass.end(), state_.get<double>("m").begin());
            std::copy(ext_tau.begin(), ext_tau.end(), state_.get<double>("external_torque").begin());
            std::copy(tau_d.begin(), tau_d.end(), state_.get<double>("torque").begin());
            std::copy(T.begin(), T.end(), state_.get<double>("T").begin());
            std::copy(state.q.begin(), state.q.end(), state_.get<double>("position").begin());
            std::copy(state.dq.begin(), state.dq.end(), state_.get<double>("velocity").begin());
            std::copy(coriolis.begin(), coriolis.end(), state_.get<double>("c").begin());
            std::copy(gravity.begin(), gravity.end(), state_.get<double>("g").begin());

            // auto &q = state_.get<double>("position");
            // auto &dq = state_.get<double>("velocity");
            // int n = robot_.dof;
            // Eigen::MatrixXd M, C, Jb, dJb, dM;
            // Eigen::VectorXd g;
            // Eigen::Matrix4d Tb, dTb;
            // std::vector<double> cmd(n);
            // m_c_g_matrix(&robot_, q, dq, M, C, g, Jb, dJb, dM, dTb, Tb);
            // double error = (g - Eigen::Map<Eigen::Vector7d>(gravity.data())).norm() / g.norm();
            // double error = (logR(Tb.block<3,3>(0,0).transpose() * Eigen::Map<Eigen::Matrix4d>(T.data()).block<3,3>(0, 0))).norm();
            // std::cerr << "error:" << error << std::endl;
            // std::cerr << "ddq:" << ddq[0] << " " << ddq[1] << " " << ddq[2] << " " << ddq[3] << " " << ddq[4] << " " << ddq[5] << std::endl;
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {

            if (RobotInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
                if (robot_ip_.empty())
                {
                    RCLCPP_ERROR(node_->get_logger(), "robot_ip is not set");
                    return CallbackReturn::FAILURE;
                }
                try
                {
                    franka_robot_ = std::make_shared<franka::Robot>(robot_ip_);
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "can not establish connection with FC3 robot with %s", robot_ip_.c_str());
                    return CallbackReturn::FAILURE;
                }
                return CallbackReturn::SUCCESS;
            }

            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {

            RobotInterface::on_shutdown(previous_state);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
            {
                // to do
                franka_robot_->automaticErrorRecovery();
                franka_robot_->setCollisionBehavior(
                    {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                    {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
                franka_robot_->setLoad(0.3, {0, 0, 0.02}, {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6});
                franka_robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 200, 200}});
                franka_robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
                control_ = franka_robot_->startTorqueControl();
                franka_model_ = std::make_shared<franka::Model>(franka_robot_->loadModel());
                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            franka_robot_->stop();
            control_ = nullptr;
            RobotInterface::on_deactivate(previous_state);
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::shared_ptr<franka::Robot> franka_robot_;
        std::unique_ptr<franka::ActiveControlBase> control_;
        std::shared_ptr<franka::Model> franka_model_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::FC3Robot, hardware_interface::RobotInterface)