#include "robot_hardware_interface/sensor_interface.hpp"
#include <iostream>
#include <vector>
#include <termios.h>
#include <fcntl.h>
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace hardwares
{
    class FTKunweiSensor : public hardware_interface::SensorInterface
    {
    public:
        FTKunweiSensor() : handle_(-1)
        {
        }
        ~FTKunweiSensor()
        {
            if (handle_ >= 0)
            {
                write_command("\x43\xAA\x0D\x0A");
                close(handle_);
            }
        }
        int write_command(const char *writedata)
        {
            int len = 0, total_len = 0;
            while (total_len < 4)
            {
                len = ::write(handle_, &writedata[total_len], 4 - total_len);
                if (len > 0)
                    total_len += len;
                else
                    return -1;
            }
            return total_len;
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (hardware_interface::SensorInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                std::string device;
                node_->get_parameter_or<std::string>("device", device, "");
                struct termios OnesensorTermios;
                if (!device.empty())
                    handle_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
                else
                    handle_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (handle_ < 0)
                {
                    RCLCPP_WARN(node_->get_logger(), "Open serial port failed! Please check the permission with ls -la /dev/ttyUSB0");
                    return CallbackReturn::FAILURE;
                }
                memset(&OnesensorTermios, 0, sizeof(OnesensorTermios));
                cfmakeraw(&OnesensorTermios);
                OnesensorTermios.c_cflag = B460800;
                OnesensorTermios.c_cflag |= CLOCAL | CREAD;
                OnesensorTermios.c_cflag &= ~CSIZE;
                OnesensorTermios.c_cflag |= CS8;
                OnesensorTermios.c_cflag &= ~PARENB;
                OnesensorTermios.c_cflag &= ~CSTOPB;
                tcflush(handle_, TCIFLUSH);
                tcflush(handle_, TCOFLUSH);
                OnesensorTermios.c_cc[VTIME] = 1;
                OnesensorTermios.c_cc[VMIN] = 1;
                tcflush(handle_, TCIFLUSH);
                tcsetattr(handle_, TCSANOW, &OnesensorTermios);
                return CallbackReturn::SUCCESS;
            }
            // publisher_ = node_->create_publisher<geometry_msgs::msg::Wrench>("~/wrench", rclcpp::SensorDataQoS());
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                stop_thread();
                write_command("\x43\xAA\x0D\x0A");
            }
            if (handle_ >= 0)
                close(handle_);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (write_command("\x48\xAA\x0D\x0A") > 0)
            {
                is_running_ = true;
                thread_ = std::make_unique<std::thread>(
                    [this]() -> void
                    {
                        std::deque<unsigned char> receiveddata;
                        unsigned char data[28];
                        union xxx
                        {
                            char m[4];
                            float n;
                        } z;
                        unsigned char readdata[150];
                        memset(readdata, 0, 150); // the length//////////////////////////////
                        int max_fd = 0;
                        fd_set readset = {0};
                        struct timeval tv = {0, 0};
                        int i, len;
                        int ReceivedDataLangth;
                        std::vector<double> force(6);
                        while (is_running_)
                        {
                            len = 0;
                            FD_ZERO(&readset);
                            FD_SET((unsigned int)handle_, &readset);
                            max_fd = handle_ + 1;
                            tv.tv_sec = 0;
                            tv.tv_usec = 0;
                            if (select(max_fd, &readset, NULL, NULL, &tv) < 0)
                            {
                                RCLCPP_WARN(node_->get_logger(), "ReadData: select error");
                                is_running_ = false;
                                continue;
                            }
                            int nRet = FD_ISSET(handle_, &readset);
                            if (nRet)
                            {
                                len = ::read(handle_, readdata, 150);
                            }
                            if (len > 0)
                            {
                                for (i = 0; i < len; i++)
                                    receiveddata.push_back(readdata[i]);
                            }

                            while (receiveddata.size() > 0)
                            {
                                ReceivedDataLangth = receiveddata.size(); // 缓存目前收到数据的长度，以免循环过程中有新数据写入或读出影响操作
                                if ((ReceivedDataLangth >= 28) && (receiveddata.at(26) == 0x0d) && (receiveddata.at(27) == 0x0a))
                                {
                                    auto force = std::make_shared<std::vector<double>>(6);
                                    for (i = 0; i < 28; i++)
                                    {
                                        data[i] = receiveddata.front();
                                        receiveddata.pop_front();
                                    }
                                    for (i = 0; i < 4; i++)
                                    {
                                        z.m[i] = data[i + 2];
                                    }
                                    force->at(0) = 1000 * z.n;
                                    for (i = 0; i < 4; i++)
                                    {
                                        z.m[i] = data[i + 6];
                                    }
                                    force->at(1) = 1000 * z.n;
                                    for (i = 0; i < 4; i++)
                                    {
                                        z.m[i] = data[i + 10];
                                    }
                                    force->at(2) = 1000 * z.n;
                                    for (i = 0; i < 4; i++)
                                    {
                                        z.m[i] = data[i + 14];
                                    }
                                    force->at(3) = 1000 * z.n;
                                    for (i = 0; i < 4; i++)
                                    {
                                        z.m[i] = data[i + 18];
                                    }
                                    force->at(4) = 1000 * z.n;
                                    for (i = 0; i < 4; i++)
                                    {
                                        z.m[i] = data[i + 22];
                                    }
                                    force->at(5) = 1000 * z.n;
                                    get<double>("force").writeFromNonRT(force);
                                    
                                    // printf("Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
                                    // geometry_msgs::msg::Wrench::UniquePtr msg = std::make_unique<geometry_msgs::msg::Wrench>();
                                    // msg->force.x = Force[0] * 1000;
                                    // msg->force.y = Force[1] * 1000;
                                    // msg->force.z = Force[2] * 1000;
                                    // msg->torque.x = Force[3] * 1000;
                                    // msg->torque.y = Force[4] * 1000;
                                    // msg->torque.z = Force[5] * 1000;
                                    // publisher_->publish(std::move(msg));
                                }
                                else if ((ReceivedDataLangth >= 28) && (ReceivedDataLangth < 120))
                                {
                                    if (receiveddata.at(0) == 0x0a)
                                        receiveddata.pop_front();
                                    else
                                    {
                                        i = 0;
                                        while ((i <= ReceivedDataLangth - 2) && (receiveddata.at(0) != 0x0d) && (receiveddata.at(1) != 0x0a))
                                        {
                                            receiveddata.pop_front();
                                            i++;
                                        }
                                        if (receiveddata.size() >= 2)
                                        {
                                            receiveddata.pop_front();
                                            receiveddata.pop_front();
                                        }
                                    }
                                }
                                else if (ReceivedDataLangth >= 120)
                                    receiveddata.clear();
                                else if (ReceivedDataLangth < 28)
                                    break;
                            }
                        }
                    });
                if(!wait_data_comming())
                    return CallbackReturn::FAILURE;
                return CallbackReturn::SUCCESS;
            }
            else
                return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
        {
            SensorInterface::on_deactivate(previous_state);
            write_command("\x43\xAA\x0D\x0A");
            return CallbackReturn::SUCCESS;
        }

    protected:
        int handle_;
        // rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::FTKunweiSensor, hardware_interface::SensorInterface)