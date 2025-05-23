#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include "robot_hardware_interface/hardware_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <atomic>
namespace hardware_interface
{

    class SensorInterface : public HardwareInterface
    {
    public:
        typedef std::shared_ptr<std::vector<double>> DataDoublePtr;
        typedef std::shared_ptr<std::vector<int>> DataIntPtr;
        typedef std::shared_ptr<std::vector<bool>> DataBoolPtr;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<DataDoublePtr>> RealtimeBufferDouble;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<DataIntPtr>> RealtimeBufferInt;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<DataBoolPtr>> RealtimeBufferBool;
        typedef std::tuple<RealtimeBufferDouble, RealtimeBufferInt, RealtimeBufferBool> RealtimeBufferType;
        using SuperClass = HardwareInterface;
        ~SensorInterface();
        SensorInterface();
        void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        void stop_thread();
        bool wait_data_comming(double t = 10);
    protected:
        template <typename T>
        realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<T>>> &get(const std::string &name)
        {
            return std::get<std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<T>>>>>(real_time_buffer_)[name];
        }

        template <typename T>
        std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<T>>>> &get()
        {
            return std::get<std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<T>>>>>(real_time_buffer_);
        }
    protected:
        int update_rate_;
        std::unique_ptr<std::thread> thread_;
        std::atomic_bool is_running_;
        RealtimeBufferType real_time_buffer_;
    };

} // namespace hardware

#endif // FT_SENSOR_INTERFACE_HPP