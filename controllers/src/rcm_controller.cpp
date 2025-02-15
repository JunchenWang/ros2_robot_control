#include "controller_interface/controller_interface.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include <iostream>
using namespace robot_math;
namespace controllers
{

    class RCMController : public controller_interface::ControllerInterface
    {
    public:
        RCMController() : d_Filter(7, 50) {}
        ~RCMController()
        {
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {

            dof_ = robot_->dof;
            p1_F_ = Eigen::Vector3d(0, 0, 0);
            p2_F_ = Eigen::Vector3d(0, 0, 0.215);
            Kn_.setZero(dof_);
            Bn_ = 1 * Eigen::VectorXd::Ones(dof_);
            Y_ = 4 * Eigen::VectorXd::Ones(dof_);
            Kx_ << 3000, 3000, 3000;
            Bx_ << 90, 90, 90;
            return CallbackReturn::SUCCESS;
        }
        Eigen::VectorXd saturate_torque(const Eigen::VectorXd &tau_d_calculated, const Eigen::VectorXd &tau_J_d, double tol = 1.0)
        {
            Eigen::VectorXd tau_d_saturated(dof_);
            for (int i = 0; i < dof_; i++)
            {
                double difference = tau_d_calculated[i] - tau_J_d[i];
                tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, tol), -tol);
            }
            return tau_d_saturated;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            time_ = 0;
            auto &T_vector = state_->get<double>("T");
            d_Filter.reset();
            Eigen::Map<const Eigen::Matrix4d> T(&T_vector[0]);
            Eigen::Vector3d p1 = T.block<3, 3>(0, 0) * p1_F_ + T.block<3, 1>(0, 3);
            Eigen::Vector3d p2 = T.block<3, 3>(0, 0) * p2_F_ + T.block<3, 1>(0, 3);
            x_d_ = p1 + 0.67442 * (p2 - p1);
            z_ = Eigen::VectorXd::Zero(dof_);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &/*t*/, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();
            auto &cmd_torque = command_->get<double>("torque");
            auto &tau_d_vector = state_->get<double>("torque");
            auto &external_torque = state_->get<double>("external_torque");
            auto &m_vector = state_->get<double>("m");
            Eigen::Map<Eigen::VectorXd> tau_c(cmd_torque.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> tau_d(tau_d_vector.data(), dof_);
            auto &q_vector = state_->get<double>("position");
            auto &dq_vector = state_->get<double>("velocity");
            auto &franka_c = state_->get<double>("c");
            auto &success_rate = state_->get<double>("success");
            Eigen::Map<const Eigen::VectorXd> coli(&franka_c[0], dof_);
            Eigen::Map<const Eigen::VectorXd> dq(&dq_vector[0], dof_);
            Eigen::Map<const Eigen::VectorXd> q(&q_vector[0], dof_);
            Eigen::Map<const Eigen::VectorXd> tau_ext(&external_torque[0], dof_);
            Eigen::Map<const Eigen::MatrixXd> m(&m_vector[0], dof_, dof_);
            std::fill(cmd_torque.begin(), cmd_torque.end(), 0);
            Eigen::MatrixXd M, C, Jb, dJb, dM;
            Eigen::VectorXd g;
            Eigen::Matrix4d Tb, dTb;
            m_c_g_matrix(robot_, q_vector, dq_vector, M, C, g, Jb, dJb, dM, dTb, Tb);
            Eigen::VectorXd P = (Y_.array() * dq.array()).matrix();
            if (time_ < 1e-5)
            {
                z_ = -P;
            }

            Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
            Eigen::Vector3d x;
            Eigen::MatrixXd J_rcm, dJ_rcm;
            rcm_Jacobian(robot_, q_vector, dq_vector, p1_F_, p2_F_, x_d_, Jb, dJb, Tb, dTb, J_rcm, dJ_rcm, x);
            Eigen::Vector3d xe = x_d_ - x;
            Eigen::VectorXd dqe = -dq;
            Eigen::Vector3d dxe = -J_rcm * dq;
            Eigen::Vector3d ddx_c = A_x_inv(J_rcm, M) * (Mu_x_X(J_rcm, M, dJ_rcm, C, dxe) + (Bx_.array() * dxe.array()).matrix() + (Kx_.array() * xe.array()).matrix());
            Eigen::VectorXd tau_x = M * J_sharp_X(J_rcm, M, ddx_c - dJ_rcm * dq);
            Eigen::VectorXd tau_n = M * null_proj(J_rcm, M, ldlt.solve((Bn_.array() * dqe.array()).matrix()));
            Eigen::VectorXd disturbance = z_ + P;
            Eigen::VectorXd proj_disturbance = M * J_sharp_X(J_rcm, M, J_rcm * ldlt.solve(disturbance));
            d_Filter.filtering(proj_disturbance.data(), proj_disturbance.data());
            tau_c = tau_x + tau_n + coli - proj_disturbance;
            Eigen::VectorXd tem = -tau_x - tau_n + proj_disturbance - P - z_;
            // update z
            Eigen::MatrixXd pinvM = pinv(M, 1e-3);
            // Eigen::VectorXd tem2 =  damping_least_square(M, tem, c, 0.1);
            z_ += (Y_.array() * (pinvM * tem).array()).matrix() * period.seconds();
            tau_c = saturate_torque(tau_c, tau_d);
        }

    protected:
        int dof_;
        Eigen::Vector3d p1_F_, p2_F_, x_d_;
        Eigen::Vector3d Kx_, Bx_;
        Eigen::VectorXd Kn_, Bn_, Y_, z_;
        MovingFilter<double> d_Filter;
        double time_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::RCMController, controller_interface::ControllerInterface)