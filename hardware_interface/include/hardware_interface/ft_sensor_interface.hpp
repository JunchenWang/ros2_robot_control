#ifndef FT_SENSOR_INTERFACE_HPP
#define FT_SENSOR_INTERFACE_HPP

#include "hardware_interface/hardware_interface.hpp"
#include "robot_math/robot_math.hpp"

namespace hardware_interface
{

    class FTSensorInterface : public HardwareInterface
    {
    public:
        ~FTSensorInterface();
        FTSensorInterface();
        void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        void compensate_gravity(const std::vector<double> &pose);
    protected:
        double mass_;
        std::vector<double> offset_;
        std::vector<double> cog_;
        std::vector<double> pose_;

    };

} // namespace hardware

#endif // FT_SENSOR_INTERFACE_HPP