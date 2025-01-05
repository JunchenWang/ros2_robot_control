#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
namespace hardware_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HardwareInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
    {
    public:
        using SharedPtr = std::shared_ptr<HardwareInterface>;
        virtual ~HardwareInterface() ;
        HardwareInterface();
        int initialize(const std::string &name, const std::string &name_space = "",
                       const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                       bool lcn_service = false);

        void finalize();
        // for simulation
        void write_state(const std::string& name, const std::vector<double> &s);
        
        virtual void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) {}
        virtual void write(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) {}
        const rclcpp_lifecycle::State &get_state() { return node_->get_current_state(); }
        CommandInterface &get_command_interface() { return command_; }
        const StateInterface &get_state_interface() { return state_; }
        const std::vector<std::string> &get_state_names() { return state_names_; }
        const std::vector<std::string> &get_command_names() { return command_names_; }
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() { return node_; }

        virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

    protected:
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::vector<std::string> state_names_;
        std::vector<std::string> command_names_;
        CommandInterface command_;
        StateInterface state_;
        std::unique_ptr<std::thread> thread_;
        volatile bool is_running_;
        realtime_tools::RealtimeBuffer<std::vector<double>> real_time_buffer_;
    };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP