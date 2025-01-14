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
        state_[state_name_] = *real_time_buffer_.readFromRT();
    }

    CallbackReturn SensorInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int len = 0;
        node_->get_parameter_or<std::string>("state_interface", state_name_, "");
        node_->get_parameter_or<int>("length", len, 0);
        if(!state_name_.empty() && len > 0)
        {
            state_[state_name_] = std::vector<double>(len, 0);
            real_time_buffer_.initRT(state_[state_name_]);
            return CallbackReturn::SUCCESS;
        }
        return CallbackReturn::FAILURE;
    }

}
