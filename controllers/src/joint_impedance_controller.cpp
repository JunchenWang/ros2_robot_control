#include "controller_interface/controller_interface.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include <iostream>
using namespace robot_math;
namespace controllers
{
    class JointImpedanceController : public controller_interface::ControllerInterface
    {
    public:
        JointImpedanceController() {}
        ~JointImpedanceController() {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            node_->get_parameter_or<std::vector<double>>("K", K_vec_, {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
            node_->get_parameter_or<std::vector<double>>("B", B_vec_, {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0});
            K_ = Eigen::Map<Eigen::VectorXd>(K_vec_.data(), dof_);
            B_ = Eigen::Map<Eigen::VectorXd>(B_vec_.data(), dof_);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            time_ = 0;
            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval();
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            q_ = Eigen::VectorXd::Zero(dof_);
            dq_ = Eigen::VectorXd::Zero(dof_);
            c_ = Eigen::VectorXd::Zero(dof_);
            c_cal_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);
            tau_d_ = Eigen::VectorXd::Zero(dof_);
            data_logger_ = new DataLogger(
                {
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(q_),
                    DATA_WRAPPER(dq_),
                    DATA_WRAPPER(c_),
                    DATA_WRAPPER(c_cal_),
                    DATA_WRAPPER(tau_cmd_),
                    DATA_WRAPPER(tau_d_),
                },
                {
                    CONFIG_WRAPPER(K_),
                    CONFIG_WRAPPER(B_),
                },
                1000);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/", "joint_impedance_controller");
            delete data_logger_;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();
            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &tau_d_vec = state_->get<double>("torque");
            const std::vector<double> &tau_ext_vec = state_->get<double>("external_torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &c_vec = state_->get<double>("c");

            tau_cmd_ = Eigen::Map<Eigen::VectorXd>(tau_cmd_vec.data(), dof_);
            tau_d_ = Eigen::Map<const Eigen::VectorXd>(tau_d_vec.data(), dof_);
            tau_ext_ = Eigen::Map<const Eigen::VectorXd>(tau_ext_vec.data(), dof_);
            q_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_);
            dq_ = Eigen::Map<const Eigen::VectorXd>(dq_vec.data(), dof_);
            c_ = Eigen::Map<const Eigen::VectorXd>(c_vec.data(), dof_);

            m_c_g_matrix(robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);
            c_cal_ = C_ * dq_;  // 验证C_ * dq与机器人提供的c是否一致

            qe_ = qd_ - q_;
            dqe_ = dqd_ - dq_;
            tau_cmd_ = M_ * ddqd_ + B_ * dqe_ + K_ * qe_ + c_;
            
            data_logger_->record();
        }

    protected:
        int dof_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_;
        Eigen::VectorXd g_, q_, dq_, c_, c_cal_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd K_, B_;
        Eigen::VectorXd tau_cmd_, tau_d_, tau_ext_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_;
        std::vector<double> K_vec_, B_vec_;
        DataLogger *data_logger_;
        double time_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::JointImpedanceController, controller_interface::ControllerInterface)