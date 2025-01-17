#include "controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include "ros2_utility/ros2_visual_tools.hpp"

#include <iostream>
using namespace robot_math;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

namespace controllers
{
    class AdmittanceController : public controller_interface::ControllerInterface
    {
    public:
        AdmittanceController() : f_tol_({0.1, 0.1, 0.1, 1, 1, 1}),
                                 inv_tol_({1e-7, 1e-5})
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

            // 声明并初始化参数 mr, br, kr
            node_->declare_parameter<double>("mr", 0.0002);
            node_->declare_parameter<double>("br", 0.018);
            node_->declare_parameter<double>("kr", 0); // 0.04

            // 声明并初始化参数 mp, bp, kp
            node_->declare_parameter<double>("mp", 0.5);
            node_->declare_parameter<double>("bp", 80);
            node_->declare_parameter<double>("kp", 0); // 100

            // 获取参数
            mr_ = node_->get_parameter("mr").as_double();
            br_ = node_->get_parameter("br").as_double();
            kr_ = node_->get_parameter("kr").as_double();
            mp_ = node_->get_parameter("mp").as_double();
            bp_ = node_->get_parameter("bp").as_double();
            kp_ = node_->get_parameter("kp").as_double();

            // 把各模型系数写进 coeff_in_box_
            coeff_in_box_.set([=](auto &value)
                              { value = {mr_, br_, kr_, mp_, bp_, kp_}; });

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
                        else if (parameter.get_name() == "mr")
                            coeff_in_box_.set([=](auto &value)
                                              { value[0] = parameter.as_double(); });
                        else if (parameter.get_name() == "br")
                            coeff_in_box_.set([=](auto &value)
                                              { value[1] = parameter.as_double(); });
                        else if (parameter.get_name() == "kr")
                            coeff_in_box_.set([=](auto &value)
                                              { value[2] = parameter.as_double(); });
                        else if (parameter.get_name() == "mp")
                            coeff_in_box_.set([=](auto &value)
                                              { value[3] = parameter.as_double(); });
                        else if (parameter.get_name() == "bp")
                            coeff_in_box_.set([=](auto &value)
                                              { value[4] = parameter.as_double(); });
                        else if (parameter.get_name() == "kp")
                            coeff_in_box_.set([=](auto &value)
                                              { value[5] = parameter.as_double(); });
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

            Tdtcp_ = pose_to_tform(state_->get<double>("pose")) * Tcp_;
            Vdtcp_.setZero();
            Tref_ = Eigen::Matrix4d::Identity();

            re_.setZero();
            red_.setZero();
            redd_.setZero();
            pe_.setZero();
            ped_.setZero();
            pedd_.setZero();

            inv_flag_ = 0;
            command_->get<int>("mode")[0] = 0;

            data_logger_ = new DataLogger(
                {
                    DATA_WRAPPER(Ftcp_),
                    DATA_WRAPPER(Fext_),
                },
                {
                    CONFIG_WRAPPER(mr_),
                    CONFIG_WRAPPER(br_),
                    CONFIG_WRAPPER(kr_),
                    CONFIG_WRAPPER(mp_),
                    CONFIG_WRAPPER(bp_),
                    CONFIG_WRAPPER(kp_),
                },
                1000);

            visual_tools_ = new ROS2VisualTools(node_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            command_->get<int>("mode")[0] = 0;
            data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/", "adm_ctrl");
            string yaml_file_path = FileUtils::getPackageDirectory("hardwares") + "/config/ur_control.yaml";
            FileUtils::modifyYamlValue(yaml_file_path, "offset", offset_);
            delete data_logger_;
            delete visual_tools_;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration &period) override
        {
            const std::vector<double> &force = com_state_->at("ft_sensor")->get<double>("force");
            const std::vector<double> &q = state_->get<double>("position");
            const std::vector<double> &dq = state_->get<double>("velocity");
            const std::vector<double> &ddq = state_->get<double>("acceleration");
            const std::vector<double> &pose = state_->get<double>("pose");
            const std::vector<bool> &io = state_->get<bool>("io");
            std::vector<double> &cmd_pose = command_->get<double>("pose");

            // 从 offset_in_box_ 中取出值给 offset_
            offset_in_box_.try_get([=](auto const &value)
                                   { offset_ = value; });
            // 从 coeff_in_box_ 中取出值给各模型系数
            coeff_in_box_.try_get([=](auto const &value)
                                  { mr_ = value[0]; br_ = value[1]; kr_ = value[2];
                                    mp_ = value[3]; bp_ = value[4]; kp_ = value[5]; });

            dq_filter_->filtering(&dq[0], &dq_filtered_[0]);
            ddq_filter_->filtering(&ddq[0], &ddq_filtered_[0]);

            Ftcp_ = gravity_and_inertia_compensation(*robot_, Tcp_, Tsensor_, q, dq_filtered_, ddq_filtered_,
                                                     &force[0], mass_ / 9.8, &offset_[0], &cog_[0], Eigen::Matrix3d::Zero());
            ftcp_filter_->filtering(Ftcp_.data(), Ftcp_.data());

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
            // 计算并更新机器人工具的目标姿态和工具实际姿态的误差 re, pe, red, ped, 便于后续导纳控制器计算 redd, pedd
            if (Ftcp_.norm() > 1e-4)
                admittance_error_cal(robot_, Tcp_, Tdtcp_, Vdtcp_, q, dq_filtered_, re_, pe_, red_, ped_, false);

            // 根据期望的导纳模型计算误差的二阶导数
            // 注意这里的pe和re定义为pd-p和rd-r，也会多出一个负号，正好和Ftcp的负号抵消
            redd_ = (Ftcp_.head(3) - br_ * red_ - kr_ * re_).array() / mr_;
            pedd_ = (Ftcp_.tail(3) - bp_ * ped_ - kp_ * pe_).array() / mp_;
            // 通过数值积分计算误差
            red_ = red_ + period.seconds() * redd_;
            re_ = re_ + period.seconds() * red_;
            ped_ = ped_ + period.seconds() * pedd_;
            pe_ = pe_ + period.seconds() * ped_;

            // 由误差计算出参考位置和姿态
            Eigen::Matrix3d R = Tdtcp_.block(0, 0, 3, 3) * exp_r(-re_);
            Eigen::Vector3d p = Tdtcp_.block(0, 3, 3, 1) - R * pe_;
            Tref_.block(0, 0, 3, 3) = R;
            Tref_.block(0, 3, 3, 1) = p;
            // 这里得到的是工具坐标系相对于基坐标系的变换矩阵，需要转化为参考末端法兰相对于基坐标系的变换矩阵
            Tref_ = Tref_ * inv_tform(Tcp_);

            command_->get<int>("mode")[0] = 2;
            cmd_pose = tform_to_pose(Tref_);

            // 修正力传感器读数，修正工具重力/重力矩及零点偏移，不考虑外力旋量和合力旋量（机器人运动）
            Fext_ = get_ext_force(force, mass_, offset_, cog_, sensor_pose_, pose_to_tform(pose));
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
            data_logger_->record();
            Eigen::Matrix4d Tatcp = pose_to_tform(pose) * Tcp_;
            visual_tools_->broadcastTransform(Tatcp, "base", "end_effector");
            visual_tools_->publishMarker(Tatcp.block(0, 3, 3, 1), "base", 0.15);
        }

    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_; // 参数回调句柄
        std::vector<double> dq_filtered_, ddq_filtered_;
        const std::vector<double> f_tol_, inv_tol_;
        double inv_flag_;
        std::vector<double> qt_;
        int dof_;
        std::vector<double> cog_, offset_, sensor_pose_;
        double mass_;
        double mr_, br_, kr_, mp_, bp_, kp_;
        realtime_tools::RealtimeBox<std::vector<double>> offset_in_box_;
        realtime_tools::RealtimeBox<std::vector<double>> coeff_in_box_;
        std::shared_ptr<MovingFilter<double>> dq_filter_, ddq_filter_, ftcp_filter_, f_filter_;
        Eigen::Vector3d re_, pe_, red_, ped_, redd_, pedd_;
        Eigen::Matrix4d Tcp_, Tsensor_, Tdtcp_, Tref_;
        Eigen::Vector6d Ftcp_, Vdtcp_, Fext_;
        bool prev_io1_;
        ROS2VisualTools *visual_tools_;
        DataLogger *data_logger_;
    };
}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::AdmittanceController, controller_interface::ControllerInterface)