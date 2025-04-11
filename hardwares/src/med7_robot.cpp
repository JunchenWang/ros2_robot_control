#include "hardware_interface/robot_interface.hpp"
#include "med7client.hpp"
#include "robot_math/robot_math.hpp"
#include <iostream>
#include <vector>

using namespace robot_math;
using namespace KUKA::FRI;
using namespace std;

namespace hardwares
{
    class Med7Robot : public hardware_interface::RobotInterface
    {
    public:
        Med7Robot()
        {
            client_ = std::make_unique<KUKA::FRI::Med7Client>();
            connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
            app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *client_);
            data_ = client_->createData();
        }
        ~Med7Robot()
        {
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            double dt = 1.0 / update_rate_;
            auto &cmd = command_.get<double>("position");
            // send command
            ESessionState currentState = (ESessionState)data_->monitoringMsg.connectionInfo.sessionState;
            switch (currentState)
            {
            case COMMANDING_WAIT:
                client_->waitCommand(state_);
                break;
            case COMMANDING_ACTIVE:
                client_->setCommand(command_);
                break;
            case IDLE:
                return;
            default:
                break;
            }
            // Encode and send command message
            data_->lastSendCounter++;
            // check if its time to send an answer
            if (data_->lastSendCounter >= data_->monitoringMsg.connectionInfo.receiveMultiplier)
            {
                data_->lastSendCounter = 0;
                // set sequence counters
                data_->commandMsg.header.sequenceCounter = data_->sequenceCounter++;
                data_->commandMsg.header.reflectedSequenceCounter =
                    data_->monitoringMsg.header.sequenceCounter;
                if (!data_->encoder.encode(data_->sendBuffer, size_))
                {
                    return;
                }
                if (!connection_->send(data_->sendBuffer, size_))
                {
                    printf("Error: failed while trying to send command message!\n");
                    return;
                }
            }
        }
        bool is_stop() override
        {
            return false;
        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            if (!connection_->isOpen())
            {
                printf("Error: client application is not connected!\n");
                return;
            }

            // Receive and decode new monitoring message
            size_ = connection_->receive(data_->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);

            if (size_ <= 0)
            {
                printf("Error: failed while trying to receive monitoring message!\n");
                return;
            }

            if (!data_->decoder.decode(data_->receiveBuffer, size_))
            {
                printf("Error: failed while trying to decode monitoring message!\n");
                return;
            }

            // check message type (so that our wrappers match)
            if (data_->expectedMonitorMsgID != data_->monitoringMsg.header.messageIdentifier)
            {
                printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
                       (int)data_->monitoringMsg.header.messageIdentifier,
                       (int)data_->expectedMonitorMsgID);
                return;
            }

            // callbacks
            // reset commmand message before callbacks
            data_->resetCommandMessage();

            // callbacks for robot client
            ESessionState currentState = (ESessionState)data_->monitoringMsg.connectionInfo.sessionState;

            if (data_->lastState != currentState)
            {
                client_->StateChange(data_->lastState, currentState, state_);
                data_->lastState = currentState;
            }

            switch (currentState)
            {
            case MONITORING_WAIT:
            case MONITORING_READY:
                client_->monitor();
                break;
            default:
                break;
            }
            client_->getState(state_);
            // for (size_t i = 0; i < state_.get<double>("position").size(); i++)
            //     std::cout << state_.get<double>("position")[i] << " ";
            // std::cout << std::endl;
            // std::cout << "read once for " << period.seconds() << std::endl;
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
                node_->get_parameter_or<int>("port", port_, 30200);
                if (robot_ip_.empty())
                {
                    RCLCPP_ERROR(node_->get_logger(), "robot_ip is not set");
                    return CallbackReturn::FAILURE;
                }
                bool flag = app_->connect(port_, robot_ip_.c_str());
                if (!flag)
                {
                    RCLCPP_ERROR(node_->get_logger(), "can not establish connection with Med7 robot with %s", robot_ip_.c_str());
                    return CallbackReturn::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "successfully connect to %s:%d", robot_ip_.c_str(), port_);
                return CallbackReturn::SUCCESS;
            }

            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            RobotInterface::on_shutdown(previous_state);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
            {
                // start control

                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            // disconnect
            app_->disconnect();
            RobotInterface::on_deactivate(previous_state);
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        int port_;
        std::unique_ptr<KUKA::FRI::ClientApplication> app_;
        std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
        std::unique_ptr<KUKA::FRI::Med7Client> client_;
        KUKA::FRI::ClientData *data_;
        int size_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::Med7Robot, hardware_interface::RobotInterface)