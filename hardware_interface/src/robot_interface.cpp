#include "hardware_interface/robot_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
namespace hardware_interface
{

    RobotInterface::RobotInterface() : dof_(0)
    {
        sensor_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SensorInterface>>("hardware_interface", "hardware_interface::SensorInterface");
    }

    int RobotInterface::configure_urdf(const std::string &robot_description)
    {
        if (!robot_description.empty() && robot_model_.initString(robot_description))
        {
            std::vector<std::string> joint_state_names, joint_command_names;
            node_->get_parameter_or<std::string>("end_effector", end_effector_, "");
            node_->get_parameter_or<std::vector<std::string>>("joint_state_interface", joint_state_names, std::vector<std::string>());
            node_->get_parameter_or<std::vector<std::string>>("joint_command_interface", joint_command_names, std::vector<std::string>());
            robot_ = robot_math::urdf_to_robot(robot_description, joint_names_, end_effector_);
            RCLCPP_INFO(node_->get_logger(), "set end_effector to be %s", end_effector_.c_str());
            dof_ = joint_names_.size();
            std::stringstream ss;
            ss << "dof: " << dof_ << " ";
            for (auto &j : joint_names_)
            {
                ss << j << " ";
            }
            RCLCPP_INFO(node_->get_logger(), "joint name in order: %s", ss.str().c_str());

            for (auto &name : joint_state_names)
            {
                state_.get<double>().emplace(std::move(name), std::vector<double>(dof_, 0.0));
            }
            for (auto &name : joint_command_names)
            {
                command_.get<double>().emplace(std::move(name), std::vector<double>(dof_, 0.0));
            }
            return 1;
        }
        return 0;
    }

    std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> RobotInterface::get_all_nodes()
    {
        std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes{node_->get_node_base_interface()};
        for (auto &p : components_)
        {
            nodes.push_back(p.second->get_node()->get_node_base_interface());
        }
        return nodes;
    }
    CallbackReturn RobotInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::string robot_description;
        node_->get_parameter_or<std::string>("robot_description", robot_description, "");
        if (!configure_urdf(robot_description))
            return CallbackReturn::FAILURE;

        // other state and command on the robot
        std::vector<std::string> commands, states;
        std::vector<long int> command_len, state_len;
        std::vector<std::string> command_type, state_type;
        node_->get_parameter_or<std::vector<std::string>>("command_interface", commands, std::vector<std::string>());
        node_->get_parameter_or<std::vector<long int>>("command_length", command_len, std::vector<long int>());
        node_->get_parameter_or<std::vector<std::string>>("command_type", command_type, std::vector<std::string>());
        if (commands.size() != command_len.size())
        {
            RCLCPP_WARN(node_->get_logger(), "command name and length are different!");
            return CallbackReturn::FAILURE;
        }

        for (int i = 0; i < commands.size(); i++)
        {
            if (command_type.empty())
                command_.get<double>().emplace(std::move(commands[i]), std::vector<double>(command_len[i], 0));
            else
            {
                if (command_type[i] == "int")
                    command_.get<int>().emplace(std::move(commands[i]), std::vector<int>(command_len[i], 0));
                else if (command_type[i] == "bool")
                    command_.get<bool>().emplace(std::move(commands[i]), std::vector<bool>(command_len[i], false));
                else
                    RCLCPP_WARN(node_->get_logger(), "command type %s is not supported!", command_type[i].c_str());
            }
        }
        node_->get_parameter_or<std::vector<std::string>>("state_interface", states, std::vector<std::string>());
        node_->get_parameter_or<std::vector<long int>>("state_length", state_len, std::vector<long int>());
        node_->get_parameter_or<std::vector<std::string>>("state_type", state_type, std::vector<std::string>());
        if (states.size() != state_len.size())
        {
            RCLCPP_WARN(node_->get_logger(), "state name and length are different!");
            return CallbackReturn::FAILURE;
        }

        for (int i = 0; i < states.size(); i++)
        {
            if (state_type.empty())
                state_.get<double>().emplace(std::move(states[i]), std::vector<double>(state_len[i], 0));
            else
            {
                if (state_type[i] == "int")
                    state_.get<int>().emplace(std::move(states[i]), std::vector<int>(state_len[i], 0));
                else if (state_type[i] == "bool")
                    state_.get<bool>().emplace(std::move(states[i]), std::vector<bool>(state_len[i], false));
                else
                    RCLCPP_WARN(node_->get_logger(), "command type %s is not supported!", state_type[i].c_str());
            }
        }

        // sensors mounted on the robot
        std::vector<std::string> sensors;
        std::string sensor_type;
        node_->get_parameter_or<std::vector<std::string>>("sensors", sensors, sensors);
        try
        {
            for (auto &sensor_name : sensors)
            {
                node_->get_parameter_or<std::string>(sensor_name, sensor_type, "");
                RCLCPP_INFO(node_->get_logger(), "found %s : %s", sensor_name.c_str(), sensor_type.c_str());
                auto sensor = sensor_loader_->createSharedInstance(sensor_type);
                components_[sensor_name] = sensor;
                com_state_.emplace(sensor_name, &sensor->get_state_interface());
                // loaned_command_.emplace(sensor_name, &sensor->get_command_interface());
                if (!sensor->initialize(sensor_name))
                {
                    throw std::runtime_error("sensor node initialized failed");
                }
            }
        }
        catch (std::exception &ex)
        {
            RCLCPP_INFO(node_->get_logger(), "%s", ex.what());
            components_.clear();
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        for (auto &c : components_)
            c.second->finalize();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        for (auto &c : components_)
            if (c.second->get_node()->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                return CallbackReturn::FAILURE;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        for (auto &c : components_)
            c.second->get_node()->deactivate();

        return CallbackReturn::SUCCESS;
    }

    void RobotInterface::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        for (auto &c : components_)
            c.second->write(t, period);
    }
    void RobotInterface::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        for (auto &c : components_)
            c.second->read(t, period);
    }
    void RobotInterface::robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t,
                                        std::function<Eigen::MatrixXd(double)> f_external,
                                        std::function<std::vector<double>(double, const std::vector<double> &, const Eigen::MatrixXd &)> controller)
    {
        int n = dof_;
        std::vector<double> q(n), dq(n), ddq(n);
        std::fill(ddq.begin(), ddq.end(), 0.0);
        std::copy(x.begin(), x.begin() + n, q.begin());
        std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());
        Eigen::MatrixXd f_ext = f_external(t);
        Eigen::VectorXd gvtao = robot_math::inverse_dynamics(&robot_, q, dq, ddq, f_ext);
        std::vector<double> cmd = controller(t, x, f_ext);
        Eigen::Map<Eigen::VectorXd> tau(&cmd[0], n);
        int m = cmd.size() - n;
        dx.resize(2 * n + m);
        std::copy(dq.begin(), dq.end(), dx.begin());
        Eigen::MatrixXd M = mass_matrix(&robot_, q);
        Eigen::VectorXd damping = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < dof_; i++)
            damping(i) = dq[i] * robot_model_.joints_.at(joint_names_[i])->dynamics->damping;
        Eigen::Map<Eigen::VectorXd>(&dx[n], n) = M.ldlt().solve(tau - gvtao - damping);
        std::copy(cmd.begin() + n, cmd.end(), dx.begin() + 2 * n);
    }
}