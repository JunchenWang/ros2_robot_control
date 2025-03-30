#include "robot_hardware_interface/hardware_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace hardware_interface
{
    HardwareInterface::HardwareInterface() 
    {
    }
    HardwareInterface::~HardwareInterface()
    {
    }
    int HardwareInterface::initialize(const std::string &name, const std::string &name_space,
                                      const rclcpp::NodeOptions &options,
                                      bool lcn_service)
    {
        rclcpp::NodeOptions node_options(options);
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            name, name_space, node_options, lcn_service); // disable LifecycleNode service interfaces or, bad_alloc exception occur!

        node_->register_on_configure(
            std::bind(&HardwareInterface::on_configure, this, std::placeholders::_1));

        node_->register_on_cleanup(
            std::bind(&HardwareInterface::on_cleanup, this, std::placeholders::_1));

        node_->register_on_activate(
            std::bind(&HardwareInterface::on_activate, this, std::placeholders::_1));

        node_->register_on_deactivate(
            std::bind(&HardwareInterface::on_deactivate, this, std::placeholders::_1));

        node_->register_on_shutdown(
            std::bind(&HardwareInterface::on_shutdown, this, std::placeholders::_1));

        node_->register_on_error(
            std::bind(&HardwareInterface::on_error, this, std::placeholders::_1));

        auto state = node_->configure();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_INFO(node_->get_logger(), "%s initialized", name.c_str());
            return 1;
        }
        return 0;
    }
    void HardwareInterface::finalize()
    {
        node_->shutdown();
    }

    void HardwareInterface::write_state(const std::string &name, const std::vector<double> &s)
    {
        state_.get<double>(name) = s;
    }
    CallbackReturn HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        command_.clear();
        state_.clear();
        std::vector<std::string> command_interface, state_interface;
        std::vector<long int> command_length, state_length;
        std::vector<std::string> command_type, state_type;
        node_->get_parameter_or<std::vector<std::string>>("command_interface", command_interface, std::vector<std::string>());
        node_->get_parameter_or<std::vector<long int>>("command_length", command_length, std::vector<long int>());
        node_->get_parameter_or<std::vector<std::string>>("command_type", command_type, std::vector<std::string>());
        if (command_interface.size() != command_length.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "command name and lengh are different!");
            return CallbackReturn::FAILURE;
        }

        for (std::size_t i = 0; i < command_interface.size(); i++)
        {
            if (command_type.empty())
                command_.get<double>().emplace(std::move(command_interface[i]), std::vector<double>(command_length[i], 0));
            else
            {
                if (command_type[i] == "int")
                    command_.get<int>().emplace(std::move(command_interface[i]), std::vector<int>(command_length[i], 0));
                else if (command_type[i] == "bool")
                    command_.get<bool>().emplace(std::move(command_interface[i]), std::vector<bool>(command_length[i], false));
                else if (command_type[i] == "double")
                    command_.get<double>().emplace(std::move(command_interface[i]), std::vector<double>(command_length[i], 0));
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "command type %s is not supported!", command_type[i].c_str());
                    return CallbackReturn::FAILURE;
                }
            }
        }
        node_->get_parameter_or<std::vector<std::string>>("state_interface", state_interface, std::vector<std::string>());
        node_->get_parameter_or<std::vector<long int>>("state_length", state_length, std::vector<long int>());
        node_->get_parameter_or<std::vector<std::string>>("state_type", state_type, std::vector<std::string>());
        if (state_interface.size() != state_length.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "state name and length are different!");
            return CallbackReturn::FAILURE;
        }

        for (std::size_t i = 0; i < state_interface.size(); i++)
        {
            if (state_type.empty())
                state_.get<double>().emplace(std::move(state_interface[i]), std::vector<double>(state_length[i], 0));
            else
            {
                if (state_type[i] == "int")
                    state_.get<int>().emplace(std::move(state_interface[i]), std::vector<int>(state_length[i], 0));
                else if (state_type[i] == "bool")
                    state_.get<bool>().emplace(std::move(state_interface[i]), std::vector<bool>(state_length[i], false));
                else if (state_type[i] == "double")
                    state_.get<double>().emplace(std::move(state_interface[i]), std::vector<double>(state_length[i], 0));
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "state type %s is not supported!", state_type[i].c_str());
                    return CallbackReturn::FAILURE;
                }
            }
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }
}