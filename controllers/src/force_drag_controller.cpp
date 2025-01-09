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
            int dof = state_->at("position").size();
            dq_filtered_ = std::vector<double>(dof, 0);
            ddq_filtered_ = std::vector<double>(dof, 0);

            node_->get_parameter_or<std::vector<double>>("cog", cog_, {0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("offset", offset_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("pose", sensor_pose_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<double>("mass", mass_, 0.0);

            // 声明并初始化参数 mw, bw
            node_->declare_parameter<double>("mw", 0.1);
            node_->declare_parameter<double>("bw", 4);

            // 声明并初始化参数 mv, bv
            node_->declare_parameter<double>("mv", 1);
            node_->declare_parameter<double>("bv", 30);

            // 获取参数值
            mw_ = node_->get_parameter("mw").as_double();
            bw_ = node_->get_parameter("bw").as_double();
            mv_ = node_->get_parameter("mv").as_double();
            bv_ = node_->get_parameter("bv").as_double();

            node_->add_on_set_parameters_callback(
                [&](std::vector<rclcpp::Parameter> parameters) -> SetParametersResult
                {
                    for (const auto &parameter : parameters)
                    {
                        if (parameter.get_name() == "offset")
                            offset_in_box_ = parameter.as_double_array();
                        else if (parameter.get_name() == "mw")
                        {
                            coeff_in_box_.set({parameter.as_double(), bw_, mv_, bv_});
                            std::cerr << "mw: " << parameter.as_double() << std::endl;
                        }
                        else if (parameter.get_name() == "bw")
                            coeff_in_box_.set({mw_, parameter.as_double(), mv_, bv_});
                        else if (parameter.get_name() == "mv")
                            coeff_in_box_.set({mw_, bw_, parameter.as_double(), bv_});
                        else if (parameter.get_name() == "bv")
                            coeff_in_box_.set({mw_, bw_, mv_, parameter.as_double()});
                    }
                    return SetParametersResult();
                });

            offset_in_box_ = offset_;
            coeff_in_box_ = {mw_, bw_, mv_, bv_};

            Tsensor_ = pose_to_tform(sensor_pose_);
            f_filter_ = std::make_shared<MovingFilter<double>>(dof);
            dq_filter_ = std::make_shared<MovingFilter<double>>(dof);
            ddq_filter_ = std::make_shared<MovingFilter<double>>(dof);

            return CallbackReturn::SUCCESS;
        }
        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration &period) override
        {
            auto const &force = com_state_->at("ft_sensor")->at("force");
            // std::cerr << "raw force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
            auto const &q = state_->at("position");
            auto const &dq = state_->at("velocity");
            auto const &io = state_->at("io");
            auto &cmd = command_->at("velocity");
            offset_in_box_.try_get([=](auto const &value)
                                   { offset_ = value; });
            coeff_in_box_.try_get([=](auto const &value)
                                  { mw_ = value[0], bw_ = value[1], mv_ = value[2], bv_ = value[3]; });

            Mw_ = mw_ * Eigen::Matrix3d::Identity();
            Bw_ = bw_ * Eigen::Matrix3d::Identity();
            Mv_ = mv_ * Eigen::Matrix3d::Identity();
            Bv_ = bv_ * Eigen::Matrix3d::Identity();

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
                                               value[i] += c[i];
                                       });
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
            Eigen::Vector6d dV;
            dV.head(3) = (f.head(3) - Bw_ * V_.head(3)).cwiseQuotient(Mw_.diagonal());
            dV.tail(3) = (f.tail(3) - Bv_ * V_.tail(3)).cwiseQuotient(Mv_.diagonal());
            V_ = V_ + dV * period.seconds();

            Eigen::Map<Eigen::Vector6d> dq_command(&cmd[0]);
            dq_command = dx_to_dq(J_, V_, 1e6, 0.1);
        }

    protected:
        double mass_;
        std::vector<double> cog_;
        std::vector<double> offset_;
        std::vector<double> sensor_pose_; // 传感器相对于法兰的位姿
        Eigen::Matrix4d T_;
        Eigen::Matrix4d Tsensor_;
        Eigen::MatrixXd J_;
        Eigen::Vector6d V_;
        double mw_, bw_, mv_, bv_;
        Eigen::Matrix3d Mw_, Bw_, Mv_, Bv_;
        realtime_tools::RealtimeBox<std::vector<double>> offset_in_box_;
        realtime_tools::RealtimeBox<std::vector<double>> coeff_in_box_;
        std::shared_ptr<MovingFilter<double>> f_filter_, dq_filter_, ddq_filter_;
        std::vector<double> dq_filtered_, ddq_filtered_;
        std::vector<double> f_tol_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForceDragController, controller_interface::ControllerInterface)