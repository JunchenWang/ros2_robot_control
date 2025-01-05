#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include "hardware_interface/hardware_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
namespace hardware_interface
{

    class SensorInterface : public HardwareInterface
    {
    public:
        ~SensorInterface();
        SensorInterface();
        void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        //void compensate_gravity(const std::vector<double> &pose);
    protected:
        std::unique_ptr<std::thread> thread_;
        volatile bool is_running_;
        realtime_tools::RealtimeBuffer<std::vector<double>> real_time_buffer_;
        std::string interface_;
        // double mass_;
        // std::vector<double> offset_;
        // std::vector<double> cog_;
        // std::vector<double> pose_;

    };

} // namespace hardware

#endif // FT_SENSOR_INTERFACE_HPP