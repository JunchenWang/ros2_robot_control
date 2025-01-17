#include "controller_interface/controller_interface.hpp"
#include <iostream>
#include"robot_math/robot_math.hpp"
namespace controllers
{
    class TorqueController : public controller_interface::ControllerInterface
    {
    public:
        TorqueController()
        {
        }
        void update(const rclcpp::Time &/*t*/, const rclcpp::Duration &/*period*/) override
        {
            auto &cmd_torque = command_->get<double>("torque");
            auto &dq = state_->get<double>("velocity");
            int n = static_cast<int>(robot_->dof);
            for (int i = 0; i < n; i++)
                cmd_torque[i] = -dq[i];
            // std::vector<double> q(n), dq(n), ddq(n);
        // // controller_->update()
        // std::fill(ddq.begin(), ddq.end(), 0.0);
        // std::copy(x.begin(), x.begin() + n, q.begin());
        // std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());
        // Eigen::MatrixXd M, C, Jb, dJb, dM;
        // Eigen::VectorXd g;
        // Eigen::Matrix4d Tb, dTb;
        // std::vector<double> cmd(n);
        // m_c_g_matrix(robot_, q, dq, M, C, g, Jb, dJb, dM, dTb, Tb);
        // for (int i = 0; i < n; i++)
        //     cmd[i] = -dq[i];
        // std::fill(cmd.begin(), cmd.end(), 0.0);
            
        }
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::TorqueController, controller_interface::ControllerInterface)