#include "robot_controller_interface/controller_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <regex>
namespace controller_interface
{
    ControllerInterface::ControllerInterface() : command_(nullptr), state_(nullptr), robot_(nullptr)
    {
    }
    void ControllerInterface::on_param_changed(const rcl_interfaces::msg::ParameterEvent &parameter_event)
    {
        // RCLCPP_INFO(
        // node_->get_logger(), "Received parameter event from node \"%s\"",
        // parameter_event.node.c_str());
    }

    
    int ControllerInterface::initialize(const std::string &name,
                                        const std::string &name_space, const rclcpp::NodeOptions &options,
                                        bool lcn_service)
    {
        rclcpp::NodeOptions node_options(options);
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            name, name_space, node_options, lcn_service); // disable LifecycleNode service interfaces

        node_->register_on_configure(
            std::bind(&ControllerInterface::on_configure, this, std::placeholders::_1));

        node_->register_on_cleanup(
            std::bind(&ControllerInterface::on_cleanup, this, std::placeholders::_1));

        node_->register_on_activate(
            std::bind(&ControllerInterface::on_activate, this, std::placeholders::_1));

        node_->register_on_deactivate(
            std::bind(&ControllerInterface::on_deactivate, this, std::placeholders::_1));

        node_->register_on_shutdown(
            std::bind(&ControllerInterface::on_shutdown, this, std::placeholders::_1));

        node_->register_on_error(
            std::bind(&ControllerInterface::on_error, this, std::placeholders::_1));

        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
        auto cb = [this](const rcl_interfaces::msg::ParameterEvent & parameter_event) {
            std::regex re(std::string("/?") + node_->get_name()); 
            if (std::regex_match(parameter_event.node, re))
            {
                on_param_changed(parameter_event);
            }     
        };
        param_event_cb_handle_ = param_subscriber_->add_parameter_event_callback(cb);
        auto state = node_->configure();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_INFO(node_->get_logger(), "%s initilized!", name.c_str());
            return 1;
        }

        return 0;
    }

    void ControllerInterface::loan_interface(int update_rate,
                                             const robot_math::Robot *robot,
                                             hardware_interface::CommandInterface *command,
                                             const hardware_interface::StateInterface *state,
                                             std::map<std::string, hardware_interface::CommandInterface *> *com_command,
                                             const std::map<std::string, const hardware_interface::StateInterface *> *com_state)
    {
        update_rate_ = update_rate;
        command_ = command;
        state_ = state;
        robot_ = robot;
        com_command_ = com_command;
        com_state_ = com_state;
    }

    void ControllerInterface::finalize()
    {

        node_->shutdown();
    }

    CallbackReturn ControllerInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

} // namespace controller_interface
