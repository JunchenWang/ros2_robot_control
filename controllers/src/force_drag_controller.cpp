#include "robot_controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "std_srvs/srv/empty.hpp"
#include <iostream>
using namespace robot_math;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

namespace controllers
{
    class ForceDragController : public controller_interface::ControllerInterface
    {
    public:
        ForceDragController() : f_tol_{1, 1, 1, 1, 1, 1}
        {
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            f_filter_->reset();
            dq_filter_->reset();
            ddq_filter_->reset();
            V_.setZero();
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {

            return CallbackReturn::SUCCESS;
        }
        void on_param_changed(const rcl_interfaces::msg::ParameterEvent &parameter_event) override
        {
            if(parameter_event.node != std::string("/") + node_->get_name())
                return;
            for (const auto &p : parameter_event.new_parameters)
            {
                if(p.name == "offset")
                {
                    offset_in_box_ = rclcpp::Parameter::from_parameter_msg(p).as_double_array();
                }
                else if(p.name == "M")
                {
                    M_in_box_ = rclcpp::Parameter::from_parameter_msg(p).as_double_array();
                }
                else if(p.name == "B")
                {
                    B_in_box_ = rclcpp::Parameter::from_parameter_msg(p).as_double_array();
                }
            };
            for (const auto &p : parameter_event.changed_parameters)
            {
                if(p.name == "offset")
                {
                    offset_in_box_ = rclcpp::Parameter::from_parameter_msg(p).as_double_array();
                }
                else if(p.name == "M")
                {
                    M_in_box_ = rclcpp::Parameter::from_parameter_msg(p).as_double_array();
                }
                else if(p.name == "B")
                {
                    B_in_box_ = rclcpp::Parameter::from_parameter_msg(p).as_double_array();
                }
            };
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            RCLCPP_ERROR(node_->get_logger(), "hehe");
            int dof = state_->get<double>("position").size();
            dq_filtered_ = std::vector<double>(dof, 0);
            ddq_filtered_ = std::vector<double>(dof, 0);
            std::vector<double> sensor_pose;
            std::vector<double> tcp_pose;
            node_->get_parameter_or<std::vector<double>>("M", M_, {0.05, 0.05, 0.05, 0.7, 0.7, 0.7});
            node_->get_parameter_or<std::vector<double>>("B", B_, {5, 5, 5, 50, 50, 50});
            node_->get_parameter_or<std::vector<double>>("cog", cog_, {0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("offset", offset_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("sensor_pose", sensor_pose, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("tcp_pose", tcp_pose, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<double>("mass", mass_, 0.0);
            
            offset_in_box_ = offset_;
            M_in_box_ = M_;
            B_in_box_ = B_;

            T_sensor_ = pose_to_tform(sensor_pose);
            T_tcp_ = pose_to_tform(tcp_pose);
            T_tcp_inv_ = inv_tform(T_tcp_);
            T_s_tcp_ = inv_tform(T_sensor_) * T_tcp_;
            f_filter_ = std::make_shared<MovingFilter<double>>(dof);
            dq_filter_ = std::make_shared<MovingFilter<double>>(dof);
            ddq_filter_ = std::make_shared<MovingFilter<double>>(dof);
            auto cb = [=](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                          std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
            {
                auto &force = com_state_->at("ft_sensor")->get<double>("force");
                Eigen::Vector6d f = get_ext_force(force, mass_, offset_, cog_, T_sensor_, T_);
                offset_in_box_.set([=, &f](auto &value)
                                   {
                                        std::vector<double> c = {f(3), f(4), f(5), f(0), f(1), f(2)};
                                        for (int i = 0; i < 6; i++)
                                            value[i] += c[i]; });
                RCLCPP_INFO(node_->get_logger(), "offsetting zero");
            };
            offsetting_service_ = node_->create_service<std_srvs::srv::Empty>("~/offset", cb);

            return CallbackReturn::SUCCESS;
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration &period) override
        {
            auto &force = com_state_->at("ft_sensor")->get<double>("force");
            // std::cerr << "raw force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            auto &cmd = command_->get<double>("velocity");
            auto &mode = command_->get<int>("mode");
            mode[0] = 2; // speed control;
            offset_in_box_.try_get([=](auto const &value)
                                   { offset_ = value; });
            M_in_box_.try_get([=](auto const &value)
                              { M_ = value; });
            B_in_box_.try_get([=](auto const &value)
                              { B_ = value; });
            Eigen::Map<Eigen::Vector6d> M(&M_[0]);
            Eigen::Map<Eigen::Vector6d> B(&B_[0]);
            dq_filter_->filtering(&dq[0], &dq_filtered_[0]);
            jacobian_matrix(robot_, q, J_, T_);
            Eigen::Vector6d f = get_ext_force(force, mass_, offset_, cog_, T_sensor_, T_);
            Eigen::Vector6d ftcp = adjoint_T(T_s_tcp_).transpose() * f;
            Eigen::MatrixXd Jtcp = adjoint_T(T_tcp_inv_) * J_;
            for (int j = 0; j < 6; j++)
            {
                if (fabs(ftcp(j)) < f_tol_[j])
                    ftcp(j) = 0;
                else
                {
                    if (ftcp(j) > 0)
                        ftcp(j) -= f_tol_[j];
                    else
                        ftcp(j) += f_tol_[j];
                }
            }
            // std::cerr << "after force: " << f(0) << " " << f(1) << " " << f(2) << " " << f(3) << " " << f(4) << " " << f(5) << std::endl;
            //  计算工具速度旋量
            auto dqf = Eigen::Map<Eigen::Vector6d>(&dq_filtered_[0]);
            V_ = Jtcp * dqf;
            // 根据期望的导纳模型计算工具速度旋量的导数
            Eigen::Vector6d dV = ((ftcp - (B.array() * V_.array()).matrix()).array() / M.array()).matrix();
            V_ = V_ + dV * period.seconds();

            Eigen::Map<Eigen::Vector6d> dq_command(&cmd[0]);
            dq_command = damping_least_square(Jtcp, V_, 1e6, 0.1);
        }

    protected:
        double mass_;             // in N
        std::vector<double> cog_; // in sensor frame
        std::vector<double> offset_;
        realtime_tools::RealtimeBox<std::vector<double>> offset_in_box_;
        Eigen::Matrix4d T_s_tcp_;
        Eigen::Matrix4d T_tcp_;     // tcp with respect to flange
        Eigen::Matrix4d T_tcp_inv_; // T_tcp_'s inverse
        Eigen::Matrix4d T_;
        Eigen::Matrix4d T_sensor_;
        Eigen::MatrixXd J_;
        Eigen::Vector6d V_;
        std::vector<double> M_;
        realtime_tools::RealtimeBox<std::vector<double>> M_in_box_;
        std::vector<double> B_;
        realtime_tools::RealtimeBox<std::vector<double>> B_in_box_;

        std::shared_ptr<MovingFilter<double>> f_filter_, dq_filter_, ddq_filter_;
        std::vector<double> dq_filtered_, ddq_filtered_;
        std::vector<double> f_tol_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr offsetting_service_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForceDragController, controller_interface::ControllerInterface)