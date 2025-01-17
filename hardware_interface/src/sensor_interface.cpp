#include "hardware_interface/sensor_interface.hpp"

using namespace robot_math;

namespace hardware_interface
{

    SensorInterface::SensorInterface() : is_running_(false)
    {
    }
    SensorInterface::~SensorInterface()
    {
        if (thread_ && thread_->joinable())
        {
            is_running_ = false;
            thread_->join();
        }
        thread_ = nullptr;
    }

    void SensorInterface::read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/)
    {
        auto & s_d = state_.get<double>();
        if(!s_d.empty())
        {
            s_d[state_name_]= *real_time_buffer_double_.readFromRT();
            return;
        }
        auto & s_i = state_.get<int>();
        if(!s_i.empty())
        {
            s_i[state_name_]= *real_time_buffer_int_.readFromRT();
            return;
        }
        auto & s_b = state_.get<bool>();
        if(!s_b.empty())
        {
            s_b[state_name_]= *real_time_buffer_bool_.readFromRT();
            return;
        }
    }

    CallbackReturn SensorInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int len = 0;
        std::string state_type;
        node_->get_parameter_or<std::string>("state_interface", state_name_, "");
        node_->get_parameter_or<std::string>("state_type", state_type, "double");
        node_->get_parameter_or<int>("length", len, 0);
        if(!state_name_.empty() && len > 0)
        {
            if(state_type == "int")
            {
                state_.get<int>().emplace(state_name_, std::vector<int>(len, 0));
                real_time_buffer_int_.initRT(state_.get<int>(state_name_));
            }
            else if(state_type == "bool")
            {
                state_.get<bool>().emplace(state_name_, std::vector<bool>(len, 0));
                real_time_buffer_bool_.initRT(state_.get<bool>(state_name_));
            }
            else if(state_type == "double")
            {
                state_.get<double>().emplace(state_name_, std::vector<double>(len, 0));
                real_time_buffer_double_.initRT(state_.get<double>(state_name_));
            }
                
            else
            {
                RCLCPP_WARN(node_->get_logger(), "state type %s is not supported!", state_type.c_str());
                return CallbackReturn::FAILURE;
            }
            return CallbackReturn::SUCCESS;
        }
        return CallbackReturn::FAILURE;
    }

}
