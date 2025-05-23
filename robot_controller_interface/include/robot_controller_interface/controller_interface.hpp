#ifndef CONTROLLER_INTERFACE_HPP
#define CONTROLLER_INTERFACE_HPP
#include "robot_hardware_interface/command_interface.hpp"
#include "robot_hardware_interface/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_math/robot_math.hpp"
namespace controller_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class ControllerInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
    {
    public:
        using SharedPtr = std::shared_ptr<ControllerInterface>;
        ControllerInterface();
        virtual ~ControllerInterface() {}
        virtual void on_param_changed(const rcl_interfaces::msg::ParameterEvent & parameter_event);
        const std::vector<double> &get_internal_state() { return internal_state_; }

        // for simulation only
        void write_state(std::vector<double>::const_iterator s, std::vector<double>::const_iterator e)
        {
            std::copy(s, e, internal_state_.begin());
        }
        int initialize(const std::string &name,
                       const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                       bool lcn_service = false);

        void finalize();

        void loan_interface( int update_rate,
                             const robot_math::Robot *robot,
                             hardware_interface::CommandInterface *command,
                             const hardware_interface::StateInterface *state,
                             std::map<std::string, hardware_interface::CommandInterface*>* com_command,
                             const std::map<std::string, const hardware_interface::StateInterface*>* com_state);

        const rclcpp_lifecycle::State &get_node_state() const { return node_->get_current_state(); }

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() { return node_; }

        virtual void update(const rclcpp::Time &/*t*/, const rclcpp::Duration &/*period*/) {};

        virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

        virtual CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);


    protected:
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        hardware_interface::CommandInterface *command_;// robot command
        const hardware_interface::StateInterface *state_;// robot state
        const robot_math::Robot *robot_;
        std::map<std::string, hardware_interface::CommandInterface*>* com_command_; // components' command
        const std::map<std::string, const hardware_interface::StateInterface*>* com_state_; // compoents' state
        std::vector<double> internal_state_; // e.g. integration of state
        int update_rate_;
        std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_event_cb_handle_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    };

} // namespace controller_interface

#endif // HARDWARE_INTERFACE_HPP