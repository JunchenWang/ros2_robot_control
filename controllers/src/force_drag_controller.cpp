#include "controller_interface/controller_interface.hpp"
#include "robot_math/robot_math.hpp"
#include <iostream>
using namespace robot_math;
namespace controllers
{
    class ForceDragController : public controller_interface::ControllerInterface
    {
    public:
        ForceDragController()
        {
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            auto& force = com_state_->at("ft_sensor")->at("force");
            auto &q = state_->at("position");
            //auto& dq = state_->at("velocity");
            Eigen::Vector6d f(force[3], force[4], force[5], force[0], force[1], force[2]);
            f /= 10;
            Eigen::MatrixXd J;
            Eigen::Matrix4d T;
            jacobian_matrix(robot_, q, J, T);
            Eigen::Vector6d dq = dx_to_dq(J, f, 1e6, 0.1);
            
            auto & cmd = command_->at("velocity");
            cmd = {dq[0], dq[1], dq[2], dq[3], dq[4], dq[5]};
            //std::cerr << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << cmd[3] << " " << cmd[4] << " " << cmd[5] << std::endl;
            //command_->at("position") = (*js)->position;

        }

    protected:
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForceDragController, controller_interface::ControllerInterface)