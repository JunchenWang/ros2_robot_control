#include "controller_interface/controller_interface.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
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
            data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/", "rcm_controller");
            delete data_logger_;
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            my_robot_= load_robot("/home/wjc/Desktop/robot_control/panda_correct.json");
            dof_ = robot_->dof;
            p1_F_ = Eigen::Vector3d(0, 0, 0);
            p2_F_ = Eigen::Vector3d(0, 0, 0.41);
            Kn_.setZero(dof_);
            Bn_ = 1 * Eigen::VectorXd::Ones(dof_);
            Y_ = 1 * Eigen::VectorXd::Ones(dof_);
            Kx_ << 3000, 3000, 3000;
            Bx_ << 90, 90, 90;
            return CallbackReturn::SUCCESS;
        }
        Eigen::VectorXd saturate_torque(const Eigen::VectorXd &tau_d_calculated, const Eigen::VectorXd &tau_J_d)
        {
            Eigen::VectorXd tau_d_saturated(dof_);
            for (int i = 0; i < dof_; i++)
            {
                double difference = tau_d_calculated[i] - tau_J_d[i];
                tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, 1.0), -1.0);
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
            x_d_ = p1 + 0.21 / 0.41 * (p2 - p1);
            z_ = Eigen::VectorXd::Zero(dof_);
            P_ = Eigen::VectorXd::Zero(dof_);
            tau_x_ = Eigen::VectorXd::Zero(dof_);
            tau_n_ = Eigen::VectorXd::Zero(dof_);
            disturbance_ = Eigen::VectorXd::Zero(dof_);
            external_torque_ = vector<double>(dof_, 0);
            dq_vector_ = vector<double>(dof_, 0);
            cmd_torque_ = vector<double>(dof_, 0);
            data_logger_ = new DataLogger(
                {
                    // DATA_WRAPPER(z_),
                    // DATA_WRAPPER(P_),
                    // DATA_WRAPPER(tau_x_),
                    // DATA_WRAPPER(tau_n_),
                    DATA_WRAPPER(xe_norm_),
                    DATA_WRAPPER(period_),
                    DATA_WRAPPER(external_torque_),
                    DATA_WRAPPER(disturbance_),
                    DATA_WRAPPER(success_rate_),
                    DATA_WRAPPER(dq_vector_),
                    DATA_WRAPPER(cmd_torque_),
                    DATA_WRAPPER(time_),
                },
                {
                    CONFIG_WRAPPER(Y_),
                },
                1000);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();
            auto &cmd_torque = command_->get<double>("torque");
            auto &tau_d_vector = state_->get<double>("torque");
            auto &external_torque = state_->get<double>("external_torque");
            Eigen::Map<Eigen::VectorXd> tau_c(cmd_torque.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> tau_d(tau_d_vector.data(), dof_);
            auto &q_vector = state_->get<double>("position");
            auto &dq_vector = state_->get<double>("velocity");
            auto &franka_c = state_->get<double>("c");
            auto &success_rate = state_->get<double>("success");
            // std::vector<double> filtered_dq(7);
            // d_Filter.filtering(dq_vector.data(), filtered_dq.data());
            Eigen::Map<const Eigen::VectorXd> coli(&franka_c[0], dof_);
            Eigen::Map<const Eigen::VectorXd> dq(&dq_vector[0], dof_);
            Eigen::Map<const Eigen::VectorXd> q(&q_vector[0], dof_);
            std::fill(cmd_torque.begin(), cmd_torque.end(), 0);
            Eigen::MatrixXd M, C, Jb, dJb, dM;
            Eigen::VectorXd g;
            Eigen::Matrix4d Tb, dTb;

            m_c_g_matrix(&my_robot_, q_vector, dq_vector, M, C, g, Jb, dJb, dM, dTb, Tb);
            Eigen::Vector3d p1 = Tb.block<3, 3>(0, 0) * p1_F_ + Tb.block<3, 1>(0, 3);
            Eigen::Vector3d p2 = Tb.block<3, 3>(0, 0) * p2_F_ + Tb.block<3, 1>(0, 3);
            Eigen::VectorXd P = (Y_.array() * dq.array()).matrix();
            if(time_ < 1e-4)
            {
                z_ = -P;
            }
            // Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(state.tau_J_d.data());
            // Eigen::Map<Eigen::Vector7d> Tau_c(Tau_c_array.data());cmd_torque_

            Eigen::LDLT<Eigen::MatrixXd> ldlt(M);

            Eigen::Vector3d a = x_d_ - p1, b = p2 - p1, x;
            double lamda = (a.dot(b) / b.dot(b));

            Eigen::MatrixXd J_rcm, dJ_rcm;
            rcm_Jacobian(&my_robot_, q_vector, dq_vector, p1_F_, p2_F_, x_d_, Jb, dJb, Tb, dTb, J_rcm, dJ_rcm, x);
            // std::cerr << cond_matrix(J_rcm * J_rcm.transpose())(0) << std::endl;

            // std::cout << singular_values << "\n\n";
            // std::cout << cond << "\n\n";
            Eigen::Vector3d xe = x_d_ - x;
            //std::cerr << xe.norm() << " " << period.seconds() <<  std::endl;
            Eigen::VectorXd dqe = -dq;
            Eigen::Vector3d dxe = -J_rcm * dq;
            Eigen::Vector3d ddx_c = A_x_inv(J_rcm, M) * (Mu_x_X(J_rcm, M, dJ_rcm, C, dxe) - (Bx_.array() * (J_rcm * dq).array()).matrix() + (Kx_.array() * xe.array()).matrix());
            double c = 1000, lambda = 5e-2;
            Eigen::VectorXd tau_x = M * J_sharp_X(J_rcm, M, ddx_c - dJ_rcm * dq, c, lambda);
            Eigen::VectorXd tau_n = M * null_proj(J_rcm, M, ldlt.solve((Bn_.array() * dqe.array()).matrix()), c, lambda);
            // std::cerr << "tau_x: " << tau_x[0] << " " << tau_x[1] << " " << tau_x[2] << " " << tau_x[3] << " " << tau_x[4] << " " << tau_x[5] << " " << tau_x[6] << std::endl;
            // std::cerr << "tau_n: " << tau_n[0] << " " << tau_n[1] << " " << tau_n[2] << " " << tau_n[3] << " " << tau_n[4] << " " << tau_n[5] << " " << tau_n[6] << std::endl;
            Eigen::VectorXd disturbance = z_ + P;
            // std::cerr << disturbance[0] << " " << disturbance[1] << " " << disturbance[2] << " " << disturbance[3] << " " << disturbance[4] << " " << disturbance[5] << " " << disturbance[6] << std::endl;
            
            disturbance = M * J_sharp_X(J_rcm, M, J_rcm * ldlt.solve(disturbance));
            d_Filter.filtering(disturbance.data(), disturbance.data());
            // std::cerr << P.norm() << std::endl;
            tau_c = tau_x + tau_n + coli - disturbance;
            // tau_c = tau_x + tau_n + coli - disturbance;
            // Tau_c = tau_x + tau_n + C * dq - G + g;
            Eigen::VectorXd tem = -tau_x - tau_n + disturbance - P - z_;
            // update z
            z_ += (Y_.array() * ldlt.solve(tem).array()).matrix() * period.seconds();
            // std::cerr << "z: " << z_[0] << " " << z_[1] << " " << z_[2] << " " << z_[3] << " " << z_[4] << " " << z_[5] << " " << z_[6] << std::endl;
            // std::cerr << z_.norm() << std::endl;
            tau_c = saturate_torque(tau_c, tau_d);

            P_ = P;
            tau_x_ = tau_x;
            tau_n_ = tau_n;
            xe_norm_ = xe.norm();
            period_ = period.seconds();
            disturbance_ = disturbance;
            external_torque_ = external_torque;
            success_rate_ = success_rate[0];
            dq_vector_ = dq_vector;
            cmd_torque_ = cmd_torque;
            data_logger_->record();
        }

    protected:
        int dof_;
        Eigen::Vector3d p1_F_, p2_F_, x_d_;
        Eigen::Vector3d Kx_, Bx_;
        Eigen::VectorXd Kn_, Bn_, Y_, z_;
        Eigen::VectorXd P_, tau_x_, tau_n_, disturbance_;
        vector<double> external_torque_, dq_vector_, cmd_torque_;
        double xe_norm_, period_, success_rate_;
        DataLogger *data_logger_;
        MovingFilter<double> d_Filter;
        double time_;
        robot_math::Robot my_robot_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::RCMController, controller_interface::ControllerInterface)