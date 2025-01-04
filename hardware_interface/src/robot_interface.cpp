#include "hardware_interface/robot_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
namespace hardware_interface
{

    RobotInterface::RobotInterface() : dof_(0)
    {
        hardware_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::HardwareInterface>>("hardware_interface", "hardware_interface::HardwareInterface");
    }

    int RobotInterface::configure_urdf(const std::string &robot_description)
    {
        if (!robot_description.empty() && robot_model_.initString(robot_description))
        {

            robot_ = robot_math::urdf_to_robot(robot_description, joint_names_);
            for (auto &j : joint_names_)
            {
                RCLCPP_INFO(node_->get_logger(), "%s", j.c_str());
            }
            dof_ = joint_names_.size();
            RCLCPP_INFO(node_->get_logger(), "DOF: %d", dof_);
            state_names_.emplace_back("position");
            state_names_.emplace_back("velocity");
            state_names_.emplace_back("torque");

            command_names_.emplace_back("position");
            command_names_.emplace_back("velocity");
            command_names_.emplace_back("torque");
            for (auto &name : state_names_)
            {
                state_.emplace(name, std::vector<double>(dof_, 0.0));
            }
            for (auto &name : command_names_)
            {
                command_.emplace(name, std::vector<double>(dof_, 0.0));
            }
            // state_names_.emplace_back("force");
            // state_.emplace("force", std::vector<double>(6,0));
            return 1;
        }
        return 0;
    }
    // void RobotInterface::receive_wrench(const geometry_msgs::msg::Wrench::UniquePtr &msg)
    // {
    //     //printf(" Received message with address: %p\n",reinterpret_cast<std::uintptr_t>(msg.get()));
    //     real_time_buffer_force_.writeFromNonRT({msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z});
    //     //state_["force"] = {msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z};
    // }
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

        std::vector<std::string> sensors;
        node_->get_parameter_or<std::vector<std::string>>("sensors", sensors, sensors);
        try
        {
            for(auto & sensor_name : sensors)
            {
                auto sensor = hardware_loader_->createSharedInstance(sensor_name);
                int pos = sensor_name.rfind(":");
                auto node_name = sensor_name.substr(pos + 1);
                components_[node_name] = sensor;
                loaned_state_.emplace(node_name, &sensor->get_state_interface());
                loaned_command_.emplace(node_name, &sensor->get_command_interface());
                if(!sensor->initialize(node_name))
                {
                    throw std::runtime_error("sensor node initialized fail");
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

    void RobotInterface::write(const rclcpp::Time & t, const rclcpp::Duration & period)
    {
        for (auto &c : components_)
            c.second->write(t, period);
    }
    void RobotInterface::read(const rclcpp::Time & t, const rclcpp::Duration & period)
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