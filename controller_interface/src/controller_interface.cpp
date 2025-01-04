#include "controller_interface/controller_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
namespace controller_interface
{
    ControllerInterface::ControllerInterface() : command_(nullptr), state_(nullptr), robot_(nullptr)
    {
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

        auto state = node_->configure();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_INFO(node_->get_logger(), "%s initilized!", name.c_str());
            return 1;
        }

        return 0;
    }

    void ControllerInterface::loarn_interface(const robot_math::Robot *robot,
                                              hardware_interface::CommandInterface *command,
                                              const hardware_interface::StateInterface *state,
                                              std::map<std::string, hardware_interface::CommandInterface*>* loaned_command,
                                              const std::map<std::string, const hardware_interface::StateInterface*>* loaned_state)
    {
        command_ = command;
        state_ = state;
        robot_ = robot;
        loaned_command_ = loaned_command;
        loaned_state_ = loaned_state;
    }

    void ControllerInterface::finalize()
    {

        node_->shutdown();
    }

    CallbackReturn ControllerInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // if (!description_.empty())
        // {
        //     robot_ = robot_math::urdf_to_robot(description_, joint_names_);
        //     return CallbackReturn::SUCCESS;
        // }
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
