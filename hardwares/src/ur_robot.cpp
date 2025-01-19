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
            // RCLCPP_INFO(node_->get_logger(), "%ld micro sec.", period.nanoseconds() / 1000);
            hardware_interface::RobotInterface::write(t, period);
            double dt = 1.0 / update_rate_;
            // auto &cmd = command_.get<double>("velocity");
            // std::for_each(cmd.begin(),cmd.end(), [](auto &s) { std::cerr << s << " ";});
            // std::cerr << std::endl;
            // return;
            int mode = command_.get<int>("mode")[0];
            switch (mode)
            {
            case 1:
                control_interface_->servoJ(command_.get<double>("position"), 1, 1, dt, 0.05, 1000);
                break;
            case 2:
                control_interface_->servoL(command_.get<double>("pose"), 1, 1, dt, 0.05, 1000);
                break;
            case 3:
                control_interface_->speedJ(command_.get<double>("velocity"), 1.5, dt);
                break;
            }
        }
        bool is_stop() override
        {
            return state_.get<bool>("io")[0];
        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::read(t, period);
            auto &q = state_.get<double>("position");
            auto &io_state = state_.get<bool>("io");
            auto &dq = state_.get<double>("velocity");
            auto &ddq = state_.get<double>("acceleration");
            auto &pose = state_.get<double>("pose");
            // auto &force = com_state_["ft_sensor"]->get<double>("force");
            // std::for_each(force.begin(), force.end(), [](auto it) { std::cerr << it << " ";});
            // std::cerr << std::endl;
            auto dt = period.seconds();

            q = receive_interface_->getActualQ();
            dq = receive_interface_->getActualQd();
            pose = receive_interface_->getActualTCPPose();

            io_state[0] = receive_interface_->getDigitalOutState(0);
            io_state[1] = receive_interface_->getDigitalOutState(1);
            if (dt > 0.5 / update_rate_)
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
            // std::cerr << "ddq:" << ddq[0] << " " << ddq[1] << " " << ddq[2] << " " << ddq[3] << " " << ddq[4] << " " << ddq[5] << std::endl;
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
            if (control_interface_ && control_interface_->isConnected())
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
                // control_interface_->stopScript();
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