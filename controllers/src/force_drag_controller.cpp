#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
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

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            int dof = state_->get<double>("position").size();
            dq_filtered_ = std::vector<double>(dof, 0);
            ddq_filtered_ = std::vector<double>(dof, 0);
            node_->get_parameter_or<std::vector<double>>("M", M_, {1, 1, 1, 1, 1, 1});
            node_->get_parameter_or<std::vector<double>>("B", B_, {1, 1, 1, 1, 1, 1});
            node_->get_parameter_or<std::vector<double>>("cog", cog_, {0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("offset", offset_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("pose", sensor_pose_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<double>("mass", mass_, 0.0);
            param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
            cb_handles_.clear();
            // Set a callback for this node's integer parameter, "an_int_param"
            cb_handles_.push_back(param_subscriber_->add_parameter_callback("offset", [this](const rclcpp::Parameter &p)
            {
                offset_in_box_ = p.as_double_array();
                RCLCPP_INFO(node_->get_logger(), "change offset");
            }));
            cb_handles_.push_back(param_subscriber_->add_parameter_callback("M", [this](const rclcpp::Parameter &p)
            {
                M_in_box_ = p.as_double_array();
                RCLCPP_INFO(node_->get_logger(), "change M");
            }));
            cb_handles_.push_back(param_subscriber_->add_parameter_callback("B", [this](const rclcpp::Parameter &p)
            {
                B_in_box_ = p.as_double_array();
                RCLCPP_INFO(node_->get_logger(), "change B");
            }));

            offset_in_box_ = offset_;
            M_in_box_ = M_;
            B_in_box_ = B_;

            Tsensor_ = pose_to_tform(sensor_pose_);
            f_filter_ = std::make_shared<MovingFilter<double>>(dof);
            dq_filter_ = std::make_shared<MovingFilter<double>>(dof);
            ddq_filter_ = std::make_shared<MovingFilter<double>>(dof);

            return CallbackReturn::SUCCESS;
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration &period) override
        {
            auto const &force = com_state_->at("ft_sensor")->get<double>("force");
            std::cerr << "raw force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
            auto const &q = state_->get<double>("position");
            auto const &dq = state_->get<double>("velocity");
            auto const &io = state_->get<double>("io");
            auto &cmd = command_->get<double>("velocity");
            offset_in_box_.try_get([=](auto const &value)
                                   { offset_ = value; });
            M_in_box_.try_get([=](auto const &value)
                              { M_ = value; });
            B_in_box_.try_get([=](auto const &value)
                              { B_ = value; });

            Eigen::Map<Eigen::Vector6d> M(&M_[0]);
            Eigen::Map<Eigen::Vector6d> B(&B_[0]);

            dq_filter_->filtering(&dq[0], &dq_filtered_[0]);
            // for (int i = 0; i < 6; i++)
            // {
            //     ddq_[i] = (dq[i] - pre_dq_[i]) / dt_;
            //     pre_dq_[i] = dq[i];
            // }
            // ddq_filter_->filtering(&ddq_[0], &ddq_filtered_[0]);
            jacobian_matrix(robot_, q, J_, T_);
            Eigen::Vector6d f = get_ext_force(force, mass_, offset_, cog_, sensor_pose_, T_);
            if (io[1])
            {
                offset_in_box_.try_set([=](auto &value)
                                       {
                                           std::cerr << "haha\n";
                                           std::vector<double> c = {f(3), f(4), f(5), f(0), f(1), f(2)};
                                           for (int i = 0; i < 6; i++)
                                               value[i] += c[i]; });
            }
            // std::cerr << f(0) << f(1) << f(2) << f(3) << f(4) << f(5) << std::endl;
            // Eigen::Matrix3d mI = Eigen::Matrix3d::Zero();
            //  Ftcp_ = gravity_and_inertia_compensation(*robot_, Tcp_, Tsensor_, q_, qd_, qdd_,
            //                                           &force_[0], mass_ / 9.8, &offset_[0], &cog_[0], mI, 1.0);
            //  Ftcp_ = -Ftcp_;

            // Ftcp_ 过小的部分置零
            for (int j = 0; j < 6; j++)
            {
                if (fabs(f(j)) < f_tol_[j])
                    f(j) = 0;
                else
                {
                    if (f(j) > 0)
                        f(j) -= f_tol_[j];
                    else
                        f(j) += f_tol_[j];
                }
            }
            std::cerr << "after force: " << f(0) << " " << f(1) << " " << f(2) << " " << f(3) << " " << f(4) << " " << f(5) << std::endl;
            // 计算工具速度旋量
            auto dqf = Eigen::Map<Eigen::Vector6d>(&dq_filtered_[0]);
            V_ = J_ * dqf;
            // 根据期望的导纳模型计算工具速度旋量的导数
            Eigen::Vector6d dV = ((f - (B.array() * V_.array()).matrix()).array() / M.array()).matrix();
            V_ = V_ + dV * period.seconds();

            Eigen::Map<Eigen::Vector6d> dq_command(&cmd[0]);
            dq_command = dx_to_dq(J_, V_, 1e6, 0.1);
        }

    protected:
        double mass_;
        std::vector<double> cog_;
        std::vector<double> offset_;
        realtime_tools::RealtimeBox<std::vector<double>> offset_in_box_;
        std::vector<double> sensor_pose_; // 传感器相对于法兰的位姿
        Eigen::Matrix4d T_;
        Eigen::Matrix4d Tsensor_;
        Eigen::MatrixXd J_;
        Eigen::Vector6d V_;
        std::vector<double> M_;
        realtime_tools::RealtimeBox<std::vector<double>> M_in_box_;
        std::vector<double> B_;
        realtime_tools::RealtimeBox<std::vector<double>> B_in_box_;

        std::shared_ptr<MovingFilter<double>> f_filter_, dq_filter_, ddq_filter_;
        std::vector<double> dq_filtered_, ddq_filtered_;
        std::vector<double> f_tol_;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForceDragController, controller_interface::ControllerInterface)