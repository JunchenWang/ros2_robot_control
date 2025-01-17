#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include "hardware_interface/hardware_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <atomic>
namespace hardware_interface
{

    class SensorInterface : public HardwareInterface
    {
    public:
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<double>>> RealtimeBufferDouble;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<int>>> RealtimeBufferInt;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<bool>>> RealtimeBufferBool;
        typedef std::tuple<RealtimeBufferDouble, RealtimeBufferInt, RealtimeBufferBool> RealtimeBufferType;
        using SuperClass = HardwareInterface;
        ~SensorInterface();
        SensorInterface();
        void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    protected:
        template <typename T>
        realtime_tools::RealtimeBuffer<std::vector<T>> &get(const std::string &name)
        {
            return std::get<std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<T>>>>(real_time_buffer_)[name];
        }

        template <typename T>
        std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<T>>> &get()
        {
            return std::get<std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<T>>>>(real_time_buffer_);
        }
    protected:
        int update_rate_;
        std::unique_ptr<std::thread> thread_;
        std::atomic_bool is_running_;
        // volatile bool is_running_;
        RealtimeBufferType real_time_buffer_;
    };

} // namespace hardware

#endif // FT_SENSOR_INTERFACE_HPP