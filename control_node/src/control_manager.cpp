#include "control_node/control_manager.h"
#include "lifecycle_msgs/msg/state.hpp"
#include <boost/numeric/odeint.hpp>
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
using namespace boost::numeric::odeint;
namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          executor_(executor),
          running_(false),
          running_box_(false),
          keep_running_(true)
    {
        update_rate_ = this->get_parameter_or<int>("update_rate", 500);
        is_simulation_ = this->get_parameter_or<bool>("simulation", true);
        is_sim_real_time_ = this->get_parameter_or<bool>("sim_real_time", true);
        is_publish_joint_state_ = this->get_parameter_or<bool>("publish_joint_state", true);
        if (is_publish_joint_state_ || is_simulation_)
        {
            joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);
        }
        robot_description_ = this->get_parameter_or<std::string>("robot_description", "");
        if (robot_description_.empty())
            throw std::runtime_error("robot description file is empty!");

        std::string robot_class = this->get_parameter_or<std::string>("robot", "");
        std::vector<std::string> controller_class = this->get_parameter_or<std::vector<std::string>>("controllers", std::vector<std::string>());
        robot_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::RobotInterface>>("hardware_interface", "hardware_interface::RobotInterface");
        controller_loader_ = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>("controller_interface", "controller_interface::ControllerInterface");
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
                controller->loan_interface(&robot_->get_robot_math(),
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

        auto stop_callback = [=](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
        {
            running_box_ = false;
        };
        stop_service_ = create_service<std_srvs::srv::Empty>("~/stop", stop_callback);
    }

    ControlManager::~ControlManager()
    {
    }

    void ControlManager::interrupt()
    {
        keep_running_ = false;
        running_box_ = false;
    }
    bool ControlManager::is_keep_running()
    {
        return keep_running_;
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
            RCLCPP_WARN(get_logger(), "controller is running, please stop first!");
            return false;
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
            response->result = activate_controller(request->cmd_params[0]);
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

    Eigen::MatrixXd ControlManager::simulation_external_force(double /*t*/)
    {
        return Eigen::MatrixXd::Zero(6, robot_->get_dof());
    }
    void ControlManager::simulation_observer(const std::vector<double> &x, double t)
    {
        // std::cerr << t << " : ";
        // for (int i = 0; i < dof_; i++)
        //     std::cerr << x[i] << " ";
        // std::cerr << "\n";
        // std::copy(x.begin(), x.begin() + dof_, joint_position_.begin());
        // std::copy(x.begin() + dof_, x.begin() + 2 * dof_, joint_velocity_.begin());
        int n = robot_->get_dof();
        if (t == 0)
        {
            Eigen::MatrixXd f_ext = simulation_external_force(t);
            simulation_controller(t, x, f_ext);
        }
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = robot_->get_joint_names();
        std::copy(x.begin(), x.begin() + n, std::back_inserter(states->position));
        std::copy(x.begin() + n, x.begin() + 2 * n, std::back_inserter(states->velocity));
        states->effort = robot_->get_command_interface().get<double>("torque");
        auto time = sim_start_time_ + rclcpp::Duration::from_seconds(t); //(std::chrono::duration<double>(t));
        // auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        states->header.stamp = time; // rclcpp::Time(nano_time.count());//this->now(); // ;
        // joint_state_publisher_->publish(*states);
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
        // wait for real time elapse
        if (is_sim_real_time_)
        {
            auto const nano_time = std::chrono::nanoseconds(time.nanoseconds());
            std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> until_time{nano_time};
            std::this_thread::sleep_until(until_time);
        }
    }

    bool ControlManager::is_simulation()
    {
        return is_simulation_;
    }

    void ControlManager::start_simulation(double time)
    {
        if (!running_)
            return;

        typedef std::vector<double> state_type;

        auto f_external = std::bind(&ControlManager::simulation_external_force, this,
                                    std::placeholders::_1);

        auto controller = std::bind(&ControlManager::simulation_controller, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3);

        auto dynamics = std::bind(&hardware_interface::RobotInterface::robot_dynamics, robot_.get(),
                                  std::placeholders::_1,
                                  std::placeholders::_2,
                                  std::placeholders::_3,
                                  std::cref(f_external), std::cref(controller));

        auto observer = std::bind(&ControlManager::simulation_observer, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2);

        // Error stepper, used to create the controlled stepper
        typedef runge_kutta_cash_karp54<state_type> error_stepper_type;
        // typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
        state_type x0(2 * robot_->get_dof(), 0);
        sim_start_time_ = this->now();
        integrate_adaptive(make_controlled(1.0e-10, 1.0e-6, error_stepper_type()), dynamics, x0, 0.0, time, 0.001, observer);
        // size_t steps = integrate_adaptive(runge_kutta4<std::vector<double>>(), dynamics, x0, 0.0, time, 0.01, observer);
    }

    std::vector<double> ControlManager::simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext)
    {
        int n = robot_->get_dof();
        std::vector<double> f{fext(0, n - 1), fext(1, n - 1), fext(2, n - 1),
                              fext(3, n - 1), fext(4, n - 1), fext(5, n - 1)};
        robot_->write_state(x, f);
        auto std_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        auto time = rclcpp::Time(std_time.count());
        auto period = rclcpp::Duration(std::chrono::duration<double>(0.0));
        active_controller_->write_state(x.begin() + 2 * n, x.end());
        active_controller_->update(time, period);
        auto cmd = robot_->get_command_interface().get<double>("torque");
        cmd.insert(cmd.end(), active_controller_->get_internal_state().begin(), active_controller_->get_internal_state().end());
        return cmd;
    }
    void ControlManager::control_loop()
    {
        // for calculating sleep time
        // double dt = 1.0 / update_rate_;
        auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
        auto const cm_now = std::chrono::nanoseconds(this->now().nanoseconds());
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
            next_iteration_time{cm_now};

        rclcpp::Time previous_time = this->now();
        while (running_ && !robot_->is_stop()) // give robot a change to stop running
        {
            // calculate measured period
            auto const current_time = this->now();
            auto const measured_period = current_time - previous_time;
            previous_time = current_time;

            // execute update loop
            read(current_time, measured_period);
            update(current_time, measured_period);
            write(current_time, measured_period);
            // get running state from box
            running_box_.try_get([=](const auto &value)
                                 { running_ = value; });

            // wait until we hit the end of the period
            next_iteration_time += period;
            std::this_thread::sleep_until(next_iteration_time);
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
            running_ = false;
            return;
        }
        robot_->get_node()->activate();
        RCLCPP_INFO(get_logger(), "waiting for controller to be activated...");
        std::stringstream ss;
        for (auto &controller : controllers_)
        {
            ss << controller->get_node()->get_name() << " ";
        }
        RCLCPP_INFO(get_logger(), "available controllers are: %s", ss.str().c_str());
        do
        {
            std::this_thread::sleep_for(1s);
            read(this->now(), rclcpp::Duration::from_seconds(1.0));
            active_controller_box_.get([=](auto const &value)
                                       { active_controller_ = value; });
        } while (keep_running_ && !active_controller_);
        if (!keep_running_)
        {
            running_ = false;
            return;
        }
        running_box_ = true;
        running_ = true;
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

        // clear robot monitor
        // rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
        //     create_client<std_srvs::srv::Empty>("robot_monitor/clear");

        // auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        // auto result = client->async_send_request(request);
    }

}
