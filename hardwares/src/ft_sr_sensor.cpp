#include "hardware_interface/sensor_interface.hpp"
#include <iostream>
#include <vector>
#include "lifecycle_msgs/msg/state.hpp"
#include "sr/sriCommDefine.h"
#include "sr/sriCommManager.h"


namespace hardwares
{

    class FTSRSensor : public hardware_interface::SensorInterface
    {
    public:
        FTSRSensor()
        {
        }
        ~FTSRSensor()
        {
            
        }

        bool on_receive_data(float fx, float fy, float fz, float mx, float my, float mz)
        {
             get<double>("force").writeFromNonRT({fx, fy, fz, mx, my, mz});
             is_data_comming_ = true;
            //std::cerr << "FTSR Sensor: " << fx << " " << fy << " " << fz << " " << mx << " " << my << " " << mz << std::endl;
            return true;
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (hardware_interface::SensorInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                std::string sensor_ip;
                int sensor_port = 0;
                node_->get_parameter_or<std::string>("sensor_ip", sensor_ip, "");
                node_->get_parameter_or<int>("sensor_port", sensor_port, 4002);
                if (!sensor_ip.empty())
                {
                    if (commManager.Init(sensor_ip, sensor_port, 
                    std::bind(&FTSRSensor::on_receive_data, this, 
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
                    std::placeholders::_4, std::placeholders::_5, std::placeholders::_6)))
                    {
                        return CallbackReturn::SUCCESS;
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "ATI sensor init failed!");
                        return CallbackReturn::FAILURE;
                    }
                    
                }
                RCLCPP_WARN(node_->get_logger(), "IP empty! ATI sensor unconfiged!");
                return CallbackReturn::FAILURE;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &/*previous_state*/) override
        {
            commManager.Stop();

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if(commManager.Run())
            {
                if(!is_data_comming())
                    return CallbackReturn::FAILURE;
                return CallbackReturn::SUCCESS;
            }
            else
                return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
        {
            SensorInterface::on_deactivate(previous_state);
            commManager.Stop();
            return CallbackReturn::SUCCESS;
        }

    protected:
        CSRICommManager commManager;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::FTSRSensor, hardware_interface::SensorInterface)