#include "controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/data_comm.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/disturbance_observer.hpp"
#include "ros2_utility/file_utils.hpp"
#include "ros2_utility/ros2_visual_tools.hpp"
#include "ros2_utility/symbolic_diffentiator.hpp"
#include <ginac/ginac.h>
#include <iostream>

using namespace robot_math;
namespace controllers
{
    class VirtualFixtureGeneralController : public controller_interface::ControllerInterface
    {
    public:
        VirtualFixtureGeneralController() : x("x"), y("y"), z("z"), x0("x0"), y0("y0"), z0("z0") {}
        ~VirtualFixtureGeneralController()
        {
            if (data_logger_)
                data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/vf_general/", "vf_general");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;

            node_->get_parameter_or<std::vector<std::string>>("Expressions", expressions_, {});
            u_num_ = expressions_.size();
            if (u_num_ == 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "No expressions are provided.");
                return CallbackReturn::FAILURE;
            }
            node_->get_parameter_or<std::vector<double>>("Ku", Ku_vec_, std::vector<double>(u_num_, 1500.0));
            node_->get_parameter_or<std::vector<double>>("Bu", Bu_vec_, std::vector<double>(u_num_, 50.0));
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, std::vector<double>(dof_, 0.0));
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, std::vector<double>(dof_, 4.0));
            node_->get_parameter_or("Y", Y_, 1.0);
            node_->get_parameter_or<std::vector<double>>("Tcp", Tcp_vec_, {0.0, 0.0, 0.0});

            Ku_ = Eigen::Map<Eigen::VectorXd>(Ku_vec_.data(), u_num_);
            Bu_ = Eigen::Map<Eigen::VectorXd>(Bu_vec_.data(), u_num_);
            Kn_ = Eigen::Map<Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<Eigen::VectorXd>(Bn_vec_.data(), dof_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            DataComm::getInstance()->setDestAddress("127.0.0.1", 7755);
            time_ = 0;
            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval(); // 深拷贝
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            // 强制修改 TCP，必须放在 on_activate 中
            Eigen::Map<Eigen::Matrix4d> TCP(const_cast<double *>(robot_->TCP));
            TCP.block(0, 3, 3, 1) = Eigen::Vector3d(Tcp_vec_[0], Tcp_vec_[1], Tcp_vec_[2]);

            Eigen::Matrix4d T;
            forward_kin_general(robot_, q_vec, T);
            R0_ = T.block(0, 0, 3, 3);
            p0_ = T.block(0, 3, 3, 1);

            if (!initializeSymbolicDifferentiators())
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to initialize symbolic differentiators.");
                return CallbackReturn::FAILURE;
            }

            Ju_ = Eigen::MatrixXd::Zero(u_num_, dof_);
            dJu_ = Eigen::MatrixXd::Zero(u_num_, dof_);
            ue_ = Eigen::VectorXd::Zero(u_num_);
            due_ = Eigen::VectorXd::Zero(u_num_);

            Thb_ = Eigen::Matrix6d::Identity();
            dThb_ = Eigen::Matrix6d::Zero();

            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(success_rate_),
                    DATA_WRAPPER(cal_time_),
                    DATA_WRAPPER(ue_),
                },
                std::initializer_list<ExperimentContext>{
                    CONFIG_WRAPPER(Ku_),
                    CONFIG_WRAPPER(Bu_),
                    CONFIG_WRAPPER(Kn_),
                    CONFIG_WRAPPER(Bn_),
                    CONFIG_WRAPPER(Y_),
                },
                1000);
            visual_tools_ = std::make_unique<ROS2VisualTools>(node_);
            disturbance_observer_ = std::make_unique<DisturbanceObserver>(Eigen::MatrixXd::Identity(dof_, dof_) * Y_, Eigen::VectorXd::Ones(dof_) * 10.0, false, 50);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();

            auto start_time = std::chrono::high_resolution_clock::now();
            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &tau_ext_vec = state_->get<double>("external_torque");
            const std::vector<double> &tau_d_vec = state_->get<double>("torque");
            success_rate_ = state_->get<double>("success")[0];

            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> tau_d(tau_d_vec.data(), dof_);

            m_c_g_matrix(robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);

            R_ = Tb_.block(0, 0, 3, 3);
            p_ = Tb_.block(0, 3, 3, 1);
            qe_ = qd_ - q;
            dqe_ = dqd_ - dq;

            Thb_.block(3, 3, 3, 3) = R_;
            dThb_.block(3, 3, 3, 3) = dTb_.block(0, 0, 3, 3);
            Jh_ = Thb_ * Jb_;
            dJh_ = dThb_ * Jb_ + Thb_ * dJb_;

            // 计算Ju, dJu, ue
            for (int i = 0; i < u_num_; i++)
            {
                const Eigen::Vector3d &grad = sym_diff_[i]->compute_gradient(p_);
                const Eigen::Matrix3d &hessian = sym_diff_[i]->compute_hessian(p_);
                Ju_.row(i) = grad.transpose() * Jh_.bottomRows(3);
                dJu_.row(i) = (Jh_.bottomRows(3) * dq).transpose() * hessian * Jh_.bottomRows(3) + grad.transpose() * dJh_.bottomRows(3);
                ue_[i] = -sym_diff_[i]->compute_value(p_);
            }
            due_ = -Ju_ * dq;

            dduc_ = Bu_.asDiagonal() * due_ + Ku_.asDiagonal() * ue_ - dJu_ * dq;
            tau_task_ = M_ * J_sharp(Ju_, M_) * dduc_;
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * null_proj(Ju_, M_, ddqd_ + ldlt.solve(Bn_.asDiagonal() * dqe_ + Kn_.asDiagonal() * qe_));
            tau_dist_ = disturbance_observer_->computeTorqueDisturbance(Ju_, dq, tau_task_ + tau_null_, M_, period.seconds());
            tau_cmd = tau_task_ + tau_null_ + C_ * dq - tau_dist_;
            tau_cmd = MathUtils::saturateTorque(tau_cmd, tau_d, 1.0);
            q_ = q;
            dq_ = dq;
            tau_cmd_ = tau_cmd;

            log2Channel(robot_data_, 0, ue_.data(), u_num_);
            log2Channel(robot_data_, 1, tau_dist_.data(), dof_);
            log2Channel(robot_data_, 2, tau_task_.data(), dof_);
            log2Channel(robot_data_, 3, tau_cmd_.data(), dof_);
            robot_data_.t = time_;
            DataComm::getInstance()->sendRobotStatus(robot_data_);
            visual_tools_->publishLineMarker(p_, "base", 1);
            data_logger_->record();
            cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        }

        bool initializeSymbolicDifferentiators()
        {
            sym_diff_.clear();
            GiNaC::parser reader({{"x", x}, {"y", y}, {"z", z}, {"x0", x0}, {"y0", y0}, {"z0", z0}});
            for (size_t i = 0; i < u_num_; i++)
            {
                try
                {
                    GiNaC::ex expr = reader(expressions_[i]);
                    expr = expr.subs({{x0, p0_[0]}, {y0, p0_[1]}, {z0, p0_[2]}});
                    sym_diff_.push_back(std::make_unique<SymbolicDifferentiator>(expr, x, y, z));
                }
                catch (const GiNaC::parse_error &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Expression parsing failed: %s (index: %zu)", e.what(), i);
                    return false;
                }
            }
            return true;
        }

    protected:
        int dof_, u_num_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_, Jh_, dJh_, Ju_, dJu_;
        Eigen::Matrix6d Thb_, dThb_;
        Eigen::VectorXd g_, q_, dq_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Ku_, Bu_, Kn_, Bn_;
        Eigen::VectorXd tau_cmd_, tau_task_, tau_null_, tau_dist_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_, ue_, due_, dduc_;
        Eigen::Matrix3d R0_, R_;
        Eigen::Vector3d p0_, p_;
        double success_rate_, cal_time_, Y_, time_;
        std::vector<double> Ku_vec_, Bu_vec_, Kn_vec_, Bn_vec_, Tcp_vec_;
        RobotData robot_data_;
        std::unique_ptr<DataLogger> data_logger_;
        std::unique_ptr<DisturbanceObserver> disturbance_observer_;
        std::unique_ptr<ROS2VisualTools> visual_tools_;
        std::vector<std::unique_ptr<SymbolicDifferentiator>> sym_diff_;
        std::vector<std::string> expressions_;
        GiNaC::symbol x, y, z, x0, y0, z0;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::VirtualFixtureGeneralController, controller_interface::ControllerInterface)