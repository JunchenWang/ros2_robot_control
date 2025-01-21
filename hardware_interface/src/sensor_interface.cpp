#include "hardware_interface/sensor_interface.hpp"

using namespace robot_math;

namespace hardware_interface
{

    SensorInterface::SensorInterface() : update_rate_(0), is_running_(false), is_data_comming_(false)
    {
    }
    SensorInterface::~SensorInterface()
    {
        stop_thread();
    }

    bool SensorInterface::is_data_comming()
    {
        auto start = node_->now();
        while (!is_data_comming_)
        {
            auto pass = node_->now() - start;
            if (pass.seconds() > 10)
            {
                RCLCPP_ERROR(node_->get_logger(), "no data comming!");
                return false;
            }
        }
        return true;
    }
    void SensorInterface::stop_thread()
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
        auto &s_d = state_.get<double>();
        for (auto &s : get<double>())
        {
            s_d[s.first] = *s.second.readFromRT();
        }
        auto &s_int = state_.get<int>();
        for (auto &s : get<int>())
        {
            s_int[s.first] = *s.second.readFromRT();
        }
        auto &s_b = state_.get<bool>();
        for (auto &s : get<bool>())
        {
            s_b[s.first] = *s.second.readFromRT();
        }
    }

    CallbackReturn SensorInterface::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
    {
        stop_thread();
        is_data_comming_ = false;
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn SensorInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        if (SuperClass::on_configure(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::FAILURE;

        node_->get_parameter_or<int>("update_rate", update_rate_, 0);
        auto &double_interface = state_.get<double>();
        auto &double_buffer = get<double>(); // std::get<0>(real_time_buffer_);
        double_buffer.clear();
        for (auto &s : double_interface)
        {
            double_buffer.emplace(s.first, s.second);
        }
        auto &int_interface = state_.get<int>();
        auto &int_buffer = get<int>(); //(real_time_buffer_);
        int_buffer.clear();
        for (auto &s : int_interface)
        {
            int_buffer.emplace(s.first, s.second);
        }
        auto &bool_interface = state_.get<bool>();
        auto &bool_buffer = get<bool>(); //(real_time_buffer_);
        bool_buffer.clear();
        for (auto &s : bool_interface)
        {
            bool_buffer.emplace(s.first, s.second);
        }

        return CallbackReturn::SUCCESS;
    }

}
