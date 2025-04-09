#include "kuka_client/IRDFClient.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;

class KukaFRINode : public rclcpp::Node
{
public:
    KukaFRINode() : Node("kuka_fri_node")
    {
        client_ = std::make_unique<KUKA::FRI::IRDFClient>();
        hw_command_mode_ = "position";
        activate();
        // 创建定时器周期调用
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            [this]()
            { read(); write(); });
        deactivate();
    }

private:
    void update_callback()
    {
        if (!client_->updateFromRobot())
        {
            RCLCPP_WARN(this->get_logger(), "从机器人读取数据失败");
            return;
        }

        auto status = client_->getRobotStatus();

        for (int i = 0; i < 7; i++)
            cout << status.measuredJointPosition[i] << " ";
        cout << endl;
        // TODO: 发布 ROS2 topic 或做其他处理

        client_->updateToRobot();
    }
    void activate()
    {
        for (auto i = 0ul; i < hw_states_external_torque_sensor_.size(); i++)
        {
            if (std::isnan(hw_states_external_torque_sensor_[i]))
            {
                hw_states_external_torque_sensor_[i] = 0;
            }
        }

        if (!client_->connect(30200, "192.170.10.2"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to KUKA robot!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to KUKA robot!");
    }

    void deactivate()
    {
        RCLCPP_INFO(this ->get_logger(), "Disconnecting from KUKA robot...");
        client_->disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected from KUKA robot!");
    }

    void read()
    {
        // read FIR and copy positions to hw_states_
        if (!client_->updateFromRobot())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to update from robot!");
            return;
        }
        if (client_->getCurrentControllerState() != KUKA::FRI::IDLE &&
            client_->getCurrentControllerState() != KUKA::FRI::MONITORING_WAIT)
        {
            client_->getRobotJointPosition(hw_states_position_);
            client_->getRobotIpoJointPosition(hw_ipo_states_position_);
            client_->getRobotJointVelocity(hw_states_velocity_);
            client_->getRobotJointTorque(hw_states_effort_);
            client_->getRobotJointExternalTorque(hw_states_external_torque_sensor_);

            if (internal_command_position[0] != internal_command_position[0])
            {
                internal_command_position = hw_states_position_;
            }
        }
    }
    void write()
    {
        hw_commands_ = hw_ipo_states_position_;
        // write hw_commands_ to FRI
        bool isNan = false;
        for (auto i = 0ul; i < hw_commands_.size(); i++)
        {
            if (hw_commands_[i] != hw_commands_[i])
            {
                isNan = true;
            }
        }

        if (client_->getCurrentControllerState() == KUKA::FRI::COMMANDING_ACTIVE && !isNan)
        {
            if (hw_command_mode_ == "position")
            {
                client_->setTargetJointPosition(hw_commands_);
            }
            else if (hw_command_mode_ == "velocity")
            {
                for (auto i = 0ul; i < hw_commands_.size(); i++)
                {
                    internal_command_position[i] = internal_command_position[i] +
                                                   client_->getRobotStatus().sampleTime * hw_commands_[i];
                }
                client_->setTargetJointPosition(internal_command_position);
            }
            else if (hw_command_mode_ == "effort")
            {
                client_->setTargetJointTorque(hw_commands_);
                client_->setTargetJointPosition(hw_states_position_);
            }
        }
        if (!client_->updateToRobot())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to update to robot!");
        }
    }

    std::unique_ptr<KUKA::FRI::IRDFClient> client_;
    std::string hw_command_mode_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_ipo_states_position_;
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_effort_;
    std::vector<double> hw_states_external_torque_sensor_;
    std::vector<double> internal_command_position;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaFRINode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
