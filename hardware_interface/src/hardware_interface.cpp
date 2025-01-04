#include "hardware_interface/hardware_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace hardware_interface
{
    HardwareInterface::HardwareInterface() : is_running_(false)
    {
    }
    HardwareInterface::~HardwareInterface()
    {
        if (thread_ && thread_->joinable())
        {
            is_running_ = false;
            thread_->join();
        }
        thread_ = nullptr;
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
        state_[name] = s;
    }
    CallbackReturn HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {

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