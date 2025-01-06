#include "controller_interface/controller_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_box.hpp"
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
        CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/) override
        {
            node_->get_parameter_or<std::vector<double>>("cog", cog_, {0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("offset", offset_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("pose", pose_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<double>("mass", mass_, 0.0);
            offset_in_box_ = offset_;
            param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
            auto cb = [this](const rclcpp::Parameter &p)
            {
                offset_in_box_ = p.as_double_array();
            };
            cb_handle_ = param_subscriber_->add_parameter_callback("offset", cb);

            return CallbackReturn::SUCCESS;
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
        {
            auto &force = com_state_->at("ft_sensor")->at("force");
            auto &q = state_->at("position");
            offset_in_box_.try_get([=](auto const &value) {offset_ = value;});
            Eigen::MatrixXd J;
            Eigen::Matrix4d T;
            jacobian_matrix(robot_, q, J, T);
            Eigen::Vector6d f = get_ext_force(force, mass_, offset_, cog_, pose_, T); 
            f /= 10;

            Eigen::Vector6d dq = dx_to_dq(J, f, 1e6, 0.1);

            auto &cmd = command_->at("velocity");
            cmd = {dq[0], dq[1], dq[2], dq[3], dq[4], dq[5]};
            // std::cerr << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << cmd[3] << " " << cmd[4] << " " << cmd[5] << std::endl;
            // command_->at("position") = (*js)->position;
        }

    protected:
        double mass_;
        std::vector<double> cog_;
        std::vector<double> offset_;
        realtime_tools::RealtimeBox<std::vector<double>> offset_in_box_;
        std::vector<double> pose_; // installed pose with respect to the endeffector
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForceDragController, controller_interface::ControllerInterface)