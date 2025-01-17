#include "hardware_interface/robot_interface.hpp"
#include "robot_math/robot_math.hpp"
#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>

using namespace robot_math;
namespace hardwares
{
    class URRobot : public hardware_interface::RobotInterface
    {
    public:
        URRobot() : pre_dq_(6)
        {
        }
        ~URRobot()
        {
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            //RCLCPP_INFO(node_->get_logger(), "%ld micro sec.", period.nanoseconds() / 1000);
            hardware_interface::RobotInterface::write(t, period);
            auto &cmd = command_.get<double>("velocity");
            // std::cerr << "cmd: " << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << cmd[3] << " " << cmd[4] << " " << cmd[5] << std::endl;
            control_interface_->speedJ(cmd, 1.5, 0.002);

        }
        bool is_stop() override
        {
            return state_.get<bool>("io")[0];
        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::read(t, period);
            auto & q = state_.get<double>("position");
            auto & dq = state_.get<double>("velocity");
            auto & ddq = state_.get<double>("acceleration");
            auto & io = state_.get<bool>("io");
            auto dt = period.seconds();
           
            auto const &force = com_state_["ft_sensor"]->get<double>("force");
            auto pose2 = receive_interface_->getActualTCPPose();
           
            q = receive_interface_->getActualQ();
            dq = receive_interface_->getActualQd();
            io[0] = receive_interface_->getDigitalOutState(0);
            io[1] = receive_interface_->getDigitalOutState(1);
            if(dt > 1e-5)
            {
                for (int i = 0; i < 6; i++)
                {
                    ddq[i] = (dq[i] - pre_dq_[i]) / dt;
                    pre_dq_[i] = dq[i];
                }
            }
            else
            {
                std::fill(ddq.begin(), ddq.end(), 0);
            }
            // Eigen::Matrix4d T;
            // forward_kin_general(&robot_, q, T);
            // auto pose = tform_to_pose(T);
            // auto pose2 = receive_interface_->getActualTCPPose();
            // auto diff = std::vector<double>(6);
            // for (int i = 0; i < 6; i++)
            // {
            //     diff[i] = pose[i] - pose2[i];
            // }
            
            
            // std::cerr << "force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
            // std::cerr << "diff: " << diff[0] << " " << diff[1] << " " << diff[2] << " " << diff[3] << " " << diff[4] << " " << diff[5] << std::endl;

            // state_["force"] = *real_time_buffer_force_.readFromRT();
            //auto force = state_["force"];
            //std::cerr << "force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
            // Eigen::Matrix4d T;
            // forward_kin_general(&robot_, state_["position"], T);

            // auto pose = tform_to_pose(T);
            // auto pose2 = receive_interface_->getActualTCPPose();
            // auto diff = std::vector<double>(6);
            // for (int i = 0; i < 6; i++)
            // {
            //     diff[i] = pose[i] - pose2[i];
            // }
            // std::cerr << "diff: " << diff[0] << " " << diff[1] << " " << diff[2] << " " << diff[3] << " " << diff[4] << " " << diff[5] << std::endl;
            // std::cerr << "pose: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4] << " " << pose[5] << std::endl;
            // std::cerr << "pose2: " << pose2[0] << " " << pose2[1] << " " << pose2[2] << " " << pose2[3] << " " << pose2[4] << " " << pose2[5] << std::endl;
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {

            if (RobotInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
                if (robot_ip_.empty())
                {
                    RCLCPP_ERROR(node_->get_logger(), "robot_ip is not set");
                    return CallbackReturn::FAILURE;
                }
                try
                {
                    control_interface_ = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip_);
                    receive_interface_ = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip_);
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "can not establish connection with UR robot with %s", robot_ip_.c_str());
                    return CallbackReturn::FAILURE;
                }

                return CallbackReturn::SUCCESS;
            }

            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            RobotInterface::on_shutdown(previous_state);
            if (control_interface_)
            {
                control_interface_->servoStop();
                control_interface_->speedStop();
                control_interface_->stopScript();
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
            {
                // to do
                std::fill(pre_dq_.begin(), pre_dq_.end(), 0);
                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            RobotInterface::on_deactivate(previous_state);
            if (control_interface_)
            {
                control_interface_->servoStop();
                control_interface_->speedStop();
                //control_interface_->stopScript();
            }
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::vector<double> pre_dq_;
        std::shared_ptr<ur_rtde::RTDEControlInterface> control_interface_;
        std::shared_ptr<ur_rtde::RTDEReceiveInterface> receive_interface_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::URRobot, hardware_interface::RobotInterface)