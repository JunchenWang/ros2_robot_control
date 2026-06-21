#include "control_node/control_manager.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
using namespace std::chrono_literals;

namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          executor_(executor)
    {
        auto parameter_file = this->get_parameter_or<std::string>("parameters", "");
        if (!parameter_file.empty())
        {
            auto path = ament_index_cpp::get_package_share_directory("applications");
            if (!path.empty())
            {
                config_ = std::make_shared<YAML::Node>();
                *config_ = YAML::LoadFile(path + "/config/" + parameter_file);
                // auto matrix = (*config_)["matrix"];
                // for(auto&& row : matrix)
                // {
                //     for(auto&& col : row)
                //     {
                //         RCLCPP_INFO(this->get_logger(), "%f", col.as<double>());
                //     }
                // }
            }
        }
        update_rate_ = this->get_parameter_or<int>("update_rate", 500);
        is_publish_joint_state_ = this->get_parameter_or<bool>("publish_joint_state", true);
        if (is_publish_joint_state_)
        {
            joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);
        }
        robot_description_ = this->get_parameter_or<std::string>("robot_description", "");
        if (robot_description_.empty())
            throw std::runtime_error("robot description file is empty!");
        
        std::string robot_class = this->get_parameter_or<std::string>("robot", "");
        std::vector<std::string> controller_class = this->get_parameter_or<std::vector<std::string>>("controllers", std::vector<std::string>());
        default_controller_ = this->get_parameter_or<std::string>("default_controller", "");
        robot_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::RobotInterface>>("robot_hardware_interface", "hardware_interface::RobotInterface");
        controller_loader_ = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>("robot_controller_interface", "controller_interface::ControllerInterface");
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        try
        {
            robot_ = robot_loader_->createSharedInstance(robot_class);
            int pos = robot_class.rfind(":");
            auto robot_name = robot_class.substr(pos + 1);
            robot_->set_update_rate(update_rate_);
            robot_->initialize(robot_name);
            auto nodes = robot_->get_all_nodes();
            for (auto &no : nodes)
                executor_->add_node(no);

            for (auto name : controller_class)
            {
                auto controller = controller_loader_->createSharedInstance(name);
                pos = name.rfind(":");
                name = name.substr(pos + 1);
                controller->loan_interface(update_rate_,
                                           &robot_->get_robot_math(),
                                           &robot_->get_command_interface(),
                                           &robot_->get_state_interface(),
                                           &robot_->get_com_command_interface(),
                                           &robot_->get_com_state_interface());
                controller->initialize(name);
                controllers_.push_back(controller);
                executor_->add_node(controller->get_node()->get_node_base_interface());
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            throw ex;
        }
        service_ = create_service<robot_control_msgs::srv::ControlCommand>("~/control_command",
                                                                           std::bind(&ControlManager::command_callback, this, std::placeholders::_1, std::placeholders::_2));

        executor_->add_node(this->get_node_base_interface());
    }

    ControlManager::~ControlManager()
    {
    }
    bool ControlManager::remove_secondary_controller(const std::string &controller_name)
    {
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        secondary_controllers_box_.set([=, &name](auto &value)
                                       {
                                           auto it = std::find_if(value.begin(), value.end(), [=](auto &&v)
                                                                  { return v->get_node()->get_name() == name; });
                                           if (it != value.end())
                                           {
                                               (*it)->get_node()->deactivate();
                                               value.erase(it);
                                           } });

        return true;
    }
    bool ControlManager::clear_secondary_controllers()
    {
        secondary_controllers_box_.set([this](auto &value)
                                       {
                                           std::for_each(value.begin(), value.end(), [=](auto &&v)
                                                         { v->get_node()->deactivate(); });
                                           value.clear(); });
        return true;
    }
    bool ControlManager::add_secondary_controller(const std::string &controller_name)
    {
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        bool ret;
        active_controller_box_.get([=, &ret, &name](const auto &value)
                                   {
                                       if (value != nullptr && value->get_node()->get_name() == name)
                                           ret = false;
                                       else
                                           ret = true; });
        if (!ret)
            return false;

        for (auto &controller : controllers_)
        {

            if (controller->get_node()->get_name() == name)
            {
                secondary_controllers_box_.set([=](auto &value)
                                               { 
                                            if(std::find(value.begin(), value.end(), controller) == value.end())
                                                value.push_back(controller); 
                                            controller->get_node()->activate(); });
                return true;
            }
        }

        return false;
    }
    bool ControlManager::load_controller(const std::string &controller_name)
    {
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        for (auto &controller : controllers_)
        {
            if (controller->get_node()->get_name() == name)
            {
                RCLCPP_INFO(get_logger(), "controller %s is already loaded!", name.c_str());
                return true;
            }
        }
        try
        {

            auto controller = controller_loader_->createSharedInstance(controller_name);
            controller->loan_interface(update_rate_,
                                       &robot_->get_robot_math(),
                                       &robot_->get_command_interface(),
                                       &robot_->get_state_interface(),
                                       &robot_->get_com_command_interface(),
                                       &robot_->get_com_state_interface());
            controller->initialize(name);
            controllers_.push_back(controller);
            executor_->add_node(controller->get_node()->get_node_base_interface());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            return false;
        }
        return true;
    }
    void ControlManager::interrupt()
    {
        keep_running_.store(false);
        running_.store(false, std::memory_order_relaxed);
    }
    bool ControlManager::is_keep_running()
    {
        return keep_running_.load();
    }
    bool ControlManager::activate_controller(const std::string &controller_name)
    {
        bool running = false;
        active_controller_box_.get([=, &running](const auto &value)
                                   {
            if (value)
                running = true; });
        if (running)
        {

            do
            {
                running_.store(false, std::memory_order_relaxed);
                std::this_thread::sleep_for(5ms);
                active_controller_box_.get([=, &running](const auto &value)
                                           {
            if (!value)
                running = false; });

            } while (running);
        }

        for (auto &controller : controllers_)
        {
            if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE && controller->get_node()->get_name() == controller_name)
            {
                bool ret = true;
                active_controller_box_.set([=, &ret, &controller](auto &value)
                                           {
                    if (value)
                    {
                        auto state = value->get_node()->deactivate();
                        if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                        {
                            ret = false;
                            return;
                        }
                    }
                    value = controller;
                    auto state = value->get_node()->activate();
                    if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                    {
                        value = nullptr;
                        ret = false;
                    } 
                    else
                        RCLCPP_INFO(get_logger(), "controller %s is activated!", controller->get_node()->get_name()); });
                return ret;
            }
        }
        return false;
    }

    void ControlManager::command_callback(const std::shared_ptr<robot_control_msgs::srv::ControlCommand::Request> request,
                                          std::shared_ptr<robot_control_msgs::srv::ControlCommand::Response> response)
    {
        std::string cmd = request->cmd_name;
        response->result = false;
        if (cmd == "activate")
            response->result = activate_controller(request->cmd_params);
        else if (cmd == "load")
            response->result = load_controller(request->cmd_params);
        else if (cmd == "add")
            response->result = add_secondary_controller(request->cmd_params);
        else if (cmd == "remove")
            response->result = remove_secondary_controller(request->cmd_params);
        else if (cmd == "clear")
            response->result = clear_secondary_controllers();
        else if (cmd == "stop")
        {
            running_.store(false, std::memory_order_relaxed);
            response->result = true;
        }
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }

    void ControlManager::shutdown_robot()
    {
        RCLCPP_INFO(this->get_logger(), "shutting down controllers and robot and all attached hardwares");
        for (auto &controller : controllers_)
        {
            controller->finalize();
        }
        robot_->finalize();
    }

    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->read(t, period);
        if (is_publish_joint_state_)
        {
            auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
            joint_state->name = robot_->get_joint_names();
            auto &state = robot_->get_state_interface().get<double>();
            auto it = state.find("position");
            if (it != state.end())
            {
                joint_state->position = it->second;
            }
            it = state.find("velocity");
            if (it != state.end())
            {
                joint_state->velocity = it->second;
            }
            it = state.find("torque");
            if (it != state.end())
            {
                joint_state->effort = it->second;
            }
            joint_state->header.stamp = t;
            // this is faster but may block the rt thread
            // joint_state_publisher_->publish(*joint_state);
            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = *joint_state;
                real_time_publisher_->unlockAndPublish();
            }
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {

        active_controller_->update(t, period);
        secondary_controllers_box_.try_get([&t, &period](const auto &value)
                                           {
            for (auto &&controller : value)
            {
                if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                {
                    controller->update(t, period);
                }
            } });
        // for (auto &controller : controllers_)
        // {
        //     if (controller->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        //     {
        //         controller->update(t, period);
        //     }
        // }
    }

    void ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->write(t, period);
    }

    void ControlManager::control_loop()
    {
        // for calculating sleep time
        // double dt = 1.0 / update_rate_;   
        bool flag = false;
        auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
        rclcpp::Time previous_time;
        rclcpp::Duration measured_period(0, 0);
        auto next_iteration_time = std::chrono::steady_clock::now();
        while (running_.load(std::memory_order_relaxed)) // give robot a change to stop running
        {
            // calculate measured period
            auto current_time = this->now();
            if (flag)
                measured_period = current_time - previous_time;
            else
                flag = true;
            previous_time = current_time;

            // execute update loop
            read(current_time, measured_period);
            update(current_time, measured_period);
            write(current_time, measured_period);

            // wait until we hit the end of the period
            next_iteration_time += period;
            const auto steady_now = std::chrono::steady_clock::now();
            if (steady_now < next_iteration_time) {
                std::this_thread::sleep_until(next_iteration_time);
            } else {
                // The loop is late. Reset the schedule to avoid accumulating delay.
                next_iteration_time = steady_now;
            }
        }
    }

    void ControlManager::prepare_loop()
    {
        auto state = robot_->get_node_state();
        while (keep_running_ && state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_WARN(this->get_logger(), "robot is not configured!");
            std::this_thread::sleep_for(1s);
        }
        if (!keep_running_)
        {
            running_.store(false, std::memory_order_relaxed);
            return;
        }
        robot_->get_node()->activate();
        RCLCPP_INFO(get_logger(), "waiting for controller to be activated...");
        std::stringstream ss;
        for (auto &&controller : controllers_)
        {
            ss << controller->get_node()->get_name() << " ";
        }
        RCLCPP_INFO(get_logger(), "available controllers are: %s", ss.str().c_str());
        std::stringstream ss2;
        secondary_controllers_box_.get([this, &ss2](const auto &value)
                                       { 
            for (auto &&controller : value)
            {
                ss2 << controller->get_node()->get_name() << " ";
            } });
        RCLCPP_INFO(get_logger(), "secondary controllers are: %s", ss2.str().c_str());
        do
        {
            std::this_thread::sleep_for(1s);
            read(this->now(), rclcpp::Duration::from_seconds(1.0));
            active_controller_box_.get([=](const auto &value)
                                       { active_controller_ = value; });
            if (!default_controller_.empty())
            {
                activate_controller(default_controller_);
                default_controller_.clear();
            }
        } while (keep_running_ && !active_controller_);
        if (!keep_running_.load())
        {
            running_.store(false, std::memory_order_relaxed);
            return;
        }
        running_.store(true, std::memory_order_relaxed);
    }

    void ControlManager::end_loop()
    {
        active_controller_box_.set([=](auto &value)
                                   {
                if (value)
                {
                    auto state = value->get_node_state();
                    if(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                        value->get_node()->deactivate();
       
                    value = nullptr;
                } });
        auto state = robot_->get_node_state();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            robot_->get_node()->deactivate();
    }

}
