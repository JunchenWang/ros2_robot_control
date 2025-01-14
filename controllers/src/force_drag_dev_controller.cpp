#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include <iostream>
using namespace robot_math;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

namespace controllers
{
    class ForceDragDevController : public controller_interface::ControllerInterface
    {
    public:
        ForceDragDevController() : f_tol_{0.5, 0.5, 0.5, 1, 1, 1}
        {
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;

            node_->get_parameter_or<std::vector<double>>("cog", cog_, {0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("offset", offset_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<std::vector<double>>("pose", sensor_pose_, {0, 0, 0, 0, 0, 0});
            node_->get_parameter_or<double>("mass", mass_, 0.0);

            // 把 offset_ 写进 offset_in_box_
            offset_in_box_.set([=](auto &value)
                               { value = offset_; });

            // 声明并初始化参数 mw, bw
            node_->declare_parameter<double>("mw", 0.05);
            node_->declare_parameter<double>("bw", 4);

            // 声明并初始化参数 mv, bv
            node_->declare_parameter<double>("mv", 0.7);
            node_->declare_parameter<double>("bv", 70);

            // 获取参数
            mw_ = node_->get_parameter("mw").as_double();
            bw_ = node_->get_parameter("bw").as_double();
            mv_ = node_->get_parameter("mv").as_double();
            bv_ = node_->get_parameter("bv").as_double();

            // 把各模型系数写进 coeff_in_box_
            coeff_in_box_.set([=](auto &value)
                              { value = {mw_, bw_, mv_, bv_}; });

            // 定义参数修改回调
            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [&](std::vector<rclcpp::Parameter> parameters) -> SetParametersResult
                {
                    RCLCPP_INFO(node_->get_logger(), "Parameter %s update requested.", parameters[0].get_name().c_str());
                    for (const auto &parameter : parameters)
                    {
                        // 根据参数名修改 box_ 中的值
                        if (parameter.get_name() == "offset")
                            offset_in_box_.set([=](auto &value)
                                               { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "mw")
                            coeff_in_box_.set([=](auto &value)
                                              { value[0] = parameter.as_double(); });
                        else if (parameter.get_name() == "bw")
                            coeff_in_box_.set([=](auto &value)
                                              { value[1] = parameter.as_double(); });
                        else if (parameter.get_name() == "mv")
                            coeff_in_box_.set([=](auto &value)
                                              { value[2] = parameter.as_double(); });
                        else if (parameter.get_name() == "bv")
                            coeff_in_box_.set([=](auto &value)
                                              { value[3] = parameter.as_double(); });
                    }
                    auto result = SetParametersResult();
                    result.successful = true;
                    return result;
                });

            // 初始化滤波器
            dq_filter_ = std::make_shared<MovingFilter<double>>(dof_);
            ddq_filter_ = std::make_shared<MovingFilter<double>>(dof_);
            ftcp_filter_ = std::make_shared<MovingFilter<double>>(dof_);
            f_filter_ = std::make_shared<MovingFilter<double>>(dof_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            prev_io1_ = false;
            dq_filtered_ = std::vector<double>(dof_, 0);
            ddq_filtered_ = std::vector<double>(dof_, 0);

            dq_filter_->reset();
            ddq_filter_->reset();
            ftcp_filter_->reset();
            f_filter_->reset();

            Tsensor_ = pose_to_tform(sensor_pose_);
            // 设置工具坐标系相对于法兰的变换矩阵Tcp，这里将工具坐标系设为与工具质心坐标系相同
            Eigen::Vector3d com(cog_[0], cog_[1], cog_[2]);                                       // 工具质心位置，在力传感器坐标系下表达
            Eigen::Vector3d Pcom = Tsensor_.block(0, 3, 3, 1) + Tsensor_.block(0, 0, 3, 3) * com; // 在法兰坐标系下的表达
            Tcp_ = Eigen::Matrix4d::Identity();
            Tcp_.block(0, 3, 3, 1) = Pcom;

            Vtcp_.setZero();
            dVtcp_.setZero();

            command_->at("mode")[0] = 0;

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            command_->at("mode")[0] = 0;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration &period) override
        {
            const std::vector<double> &force = com_state_->at("ft_sensor")->at("force");
            const std::vector<double> &q = state_->at("position");
            const std::vector<double> &dq = state_->at("velocity");
            const std::vector<double> &ddq = state_->at("acceleration");
            const std::vector<double> &pose = state_->at("pose");
            const std::vector<double> &io = state_->at("io");
            std::vector<double> &cmd_qd = command_->at("velocity");

            // 从 offset_in_box_ 中取出值给 offset_
            offset_in_box_.try_get([=](auto const &value)
                                   { offset_ = value; });

            // 从 coeff_in_box_ 中取出值给各模型系数
            coeff_in_box_.try_get([=](auto const &value)
                                  { mw_ = value[0], bw_ = value[1], mv_ = value[2], bv_ = value[3]; });

            dq_filter_->filtering(&dq[0], &dq_filtered_[0]);
            ddq_filter_->filtering(&ddq[0], &ddq_filtered_[0]);

            Ftcp_ = gravity_and_inertia_compensation(*robot_, Tcp_, Tsensor_, q, dq_filtered_, ddq_filtered_,
                                                     &force[0], mass_ / 9.8, &offset_[0], &cog_[0], Eigen::Matrix3d::Zero());
            ftcp_filter_->filtering(Ftcp_.data(), Ftcp_.data());
            // RCLCPP_INFO(node_->get_logger(), "Ftcp_: %f, %f, %f, %f, %f, %f", Ftcp_(0), Ftcp_(1), Ftcp_(2), Ftcp_(3), Ftcp_(4), Ftcp_(5));

            // 转为外界作用于工具的力旋量
            Ftcp_ = -Ftcp_;

            // Ftcp_ 过小的部分置零
            for (int j = 0; j < 6; j++)
            {
                if (fabs(Ftcp_(j)) < f_tol_[j])
                    Ftcp_(j) = 0;
                else
                {
                    if (Ftcp_(j) > 0)
                        Ftcp_(j) -= f_tol_[j];
                    else
                        Ftcp_(j) += f_tol_[j];
                }
            }
            // 计算法兰雅可比、工具雅可比和工具雅可比的逆矩阵
            jacobian_matrix(robot_, q, Jb_, T_);
            Jtcp_ = adjoint_T(inv_tform(Tcp_)) * Jb_;

            // 计算工具速度旋量
            Vtcp_ = Jtcp_ * Eigen::Map<Eigen::Vector6d>(&dq_filtered_[0]);
            // 根据期望的导纳模型计算工具速度旋量的导数
            dVtcp_.head(3) = (Ftcp_.head(3) - bw_ * Vtcp_.head(3)).array() / mw_;
            dVtcp_.tail(3) = (Ftcp_.tail(3) - bv_ * Vtcp_.tail(3)).array() / mv_;
            Vtcp_ = Vtcp_ + dVtcp_ * period.seconds();

            command_->at("mode")[0] = 3; // speedJ
            Eigen::Map<Eigen::Vector6d> cmd_qd_map(&cmd_qd[0]);
            cmd_qd_map = dx_to_dq(Jtcp_, Vtcp_, 1e6, 0.1);
            // RCLCPP_INFO(node_->get_logger(), "cmd_qd: %f, %f, %f, %f, %f, %f", cmd_qd[0], cmd_qd[1], cmd_qd[2], cmd_qd[3], cmd_qd[4], cmd_qd[5]);

            // 修正力传感器读数，修正工具重力/重力矩及零点偏移，不考虑外力旋量和合力旋量（机器人运动）
            Fext_ = get_ext_force(force, mass_, offset_, cog_, sensor_pose_, T_);
            f_filter_->filtering(Fext_.data(), Fext_.data());
            // RCLCPP_INFO(node_->get_logger(), "Fext_: %f, %f, %f, %f, %f, %f", Fext_(0), Fext_(1), Fext_(2), Fext_(3), Fext_(4), Fext_(5));
            if (!prev_io1_ && io[1])
            {
                offset_in_box_.try_set([=](auto &value)
                                       {
                                            RCLCPP_INFO(node_->get_logger(), "offset_in_box_ set!");
                                            std::vector<double> c = {Fext_(3), Fext_(4), Fext_(5),
                                                                     Fext_(0), Fext_(1), Fext_(2)};
                                            for (int i = 0; i < 6; i++)
                                                value[i] += c[i]; });
            }
            prev_io1_ = io[1];
        }

    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_; // 参数回调句柄
        std::vector<double> dq_filtered_, ddq_filtered_;
        std::vector<double> f_tol_;
        int dof_;
        std::vector<double> cog_, offset_, sensor_pose_;
        double mass_;
        double mw_, bw_, mv_, bv_;
        realtime_tools::RealtimeBox<std::vector<double>> offset_in_box_;
        realtime_tools::RealtimeBox<std::vector<double>> coeff_in_box_;
        std::shared_ptr<MovingFilter<double>> dq_filter_, ddq_filter_, ftcp_filter_, f_filter_;
        Eigen::Matrix4d Tcp_, Tsensor_, T_;
        Eigen::Vector6d Ftcp_, Vtcp_, dVtcp_, Fext_;
        Eigen::Matrix6d Jtcp_;
        Eigen::MatrixXd Jb_;
        bool prev_io1_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::ForceDragDevController, controller_interface::ControllerInterface)