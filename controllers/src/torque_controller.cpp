#include "controller_interface/controller_interface.hpp"
#include "robot_math/robot_math.hpp"
#include <iostream>
namespace controllers
{
    class TorqueController : public controller_interface::ControllerInterface
    {
    public:
        TorqueController()
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            auto &cmd_torque = command_->get<double>("torque");
            std::fill(cmd_torque.begin(), cmd_torque.end(), 0);
            // auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            int n = robot_->dof;
            for(int i = 0; i < n; i++)
                cmd_torque[i] = -dq[i];
            // Eigen::MatrixXd M, C, Jb, dJb, dM;
            // Eigen::VectorXd g;
            // Eigen::Matrix4d Tb, dTb;
            // std::vector<double> cmd(n);
            // m_c_g_matrix(robot_, q, dq, M, C, g, Jb, dJb, dM, dTb, Tb);
            //std::copy(g.data(), g.data() + n, cmd_torque.begin());
        }
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::TorqueController, controller_interface::ControllerInterface)