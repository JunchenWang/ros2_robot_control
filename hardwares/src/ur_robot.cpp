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
        URRobot()
        {
        }

        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::write(t, period);

        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::read(t, period);
            state_["position"] = receive_interface_->getActualQ();
            state_["velocity"] = receive_interface_->getActualQd();
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
                    // control_interface_ = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip_);
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
                control_interface_->stopScript();
            }
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::shared_ptr<ur_rtde::RTDEControlInterface> control_interface_;
        std::shared_ptr<ur_rtde::RTDEReceiveInterface> receive_interface_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::URRobot, hardware_interface::RobotInterface)