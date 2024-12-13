#include "control_node/control_manager.h"
#include <boost/numeric/odeint.hpp>


using namespace boost::numeric::odeint;
namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          executor_(executor),
          param_listener_(std::make_shared<ParamListener>(this->get_node_parameters_interface())),
          is_new_cmd_available_(false),
          robot_(nullptr),
          robot_model_(nullptr),
          command_(nullptr),
          state_(nullptr),
          dof_(0)
    {
        params_ = param_listener_->get_params();
        update_rate_ = params_.update_rate;
        RCLCPP_INFO(this->get_logger(), "%d", update_rate_);
        joint_command_topic_name_ = this->get_parameter_or<std::string>("cmd_name", "gui/joint_state");
        is_simulation_ = this->get_parameter_or<bool>("simulation", true);
        is_sim_real_time_ = this->get_parameter_or<bool>("sim_real_time", true);
        command_receiver_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_command_topic_name_, rclcpp::SensorDataQoS(),
                                                                                    std::bind(&ControlManager::robot_joint_command_callback, this, std::placeholders::_1));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
        description_sub_ = this->create_subscription<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local(),
                                                                            std::bind(&ControlManager::robot_description_callback, this, std::placeholders::_1));
        real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);

        robot_description_ = this->get_parameter_or<std::string>("robot_description", "");
        std::string hardware_class = this->get_parameter_or<std::string>("hardware_class", "");
        std::string controller_class = this->get_parameter_or<std::string>("controller_class", "");
        hardware_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::HardwareInterface>>("hardware_interface", "hardware_interface::HardwareInterface");
        controller_loader_ = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>("controller_interface", "controller_interface::ControllerInterface");
        try
        {
            hardware_ = hardware_loader_->createSharedInstance(hardware_class);
            controller_ = controller_loader_->createSharedInstance(controller_class);
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            throw ex;
        }
    }

    ControlManager::~ControlManager()
    {
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }

    void ControlManager::init_robot()
    {
        bool ready;
        do
        {
            RCLCPP_INFO(get_logger(), "waiting for robot");
            robot_desp_mutex_.lock();
            ready = robot_description_.empty();
            robot_desp_mutex_.unlock();
            std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        } while (ready);

        hardware_->initialize(robot_description_);
        dof_ = hardware_->get_dof();
        robot_ = &hardware_->get_robot_model();
        command_ = &hardware_->get_command_interface();
        state_ = &hardware_->get_state_interface();
        joint_names_ = hardware_->get_joint_name();
        robot_model_ = &hardware_->get_urdf_model();

        RCLCPP_INFO(get_logger(), "robot ok");
    }

    void ControlManager::shutdown_robot()
    {
        hardware_->finalize();
        dof_ = 0;
        command_ = nullptr;
        state_ = nullptr;
        robot_model_ = nullptr;
        robot_ = nullptr;
        joint_names_.clear();
    }

    void ControlManager::robot_description_callback(std_msgs::msg::String::SharedPtr desp)
    {
        std::lock_guard<std::mutex> guard(robot_desp_mutex_);
        robot_description_ = desp->data;
    }

    void ControlManager::robot_joint_command_callback(sensor_msgs::msg::JointState::SharedPtr js)
    {
        real_time_buffer_.writeFromNonRT(js);
        is_new_cmd_available_ = true;
    }

    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        hardware_->read();
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = joint_names_;
        states->position = state_->at("position");
        states->velocity = state_->at("velocity");
        states->effort = state_->at("effort");
        states->header.stamp = t;
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        controller_->update(robot_, state_, command_);
    }

    void ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        hardware_->write();
    }

    Eigen::MatrixXd ControlManager::simulation_external_force(double t)
    {
        return Eigen::MatrixXd::Zero(6, dof_);
    }
    void ControlManager::robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t)
    {
        int n = dof_;
        std::vector<double> q(n), dq(n), ddq(n);
        std::fill(ddq.begin(), ddq.end(), 0.0);
        std::copy(x.begin(), x.begin() + n, q.begin());
        std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());
        Eigen::MatrixXd f_ext = simulation_external_force(t);
        Eigen::VectorXd gvtao = robot_math::inverse_dynamics(robot_, q, dq, ddq, f_ext);
        std::vector<double> cmd = simulation_controller(t, x, f_ext);
        Eigen::Map<Eigen::VectorXd> tau(&cmd[0], n);
        int m = cmd.size() - n;
        dx.resize(2 * n + m);
        std::copy(dq.begin(), dq.end(), dx.begin());
        Eigen::MatrixXd M = mass_matrix(robot_, q);
        Eigen::VectorXd damping = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < dof_; i++)
            damping(i) = dq[i] * robot_model_->joints_.at(joint_names_[i])->dynamics->damping;
        Eigen::Map<Eigen::VectorXd>(&dx[n], n) = M.ldlt().solve(tau - gvtao - damping);
        std::copy(cmd.begin() + n, cmd.end(), dx.begin() + 2 * n);
    }
    void ControlManager::simulation_observer(const std::vector<double> &x, double t)
    {
        // std::cerr << t << " : ";
        // for (int i = 0; i < dof_; i++)
        //     std::cerr << x[i] << " ";
        // std::cerr << "\n";
        // std::copy(x.begin(), x.begin() + dof_, joint_position_.begin());
        // std::copy(x.begin() + dof_, x.begin() + 2 * dof_, joint_velocity_.begin());
        if (t == 0)
        {
            Eigen::MatrixXd f_ext = simulation_external_force(t);
            simulation_controller(t, x, f_ext);
        }
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = joint_names_;
        std::copy(x.begin(), x.begin() + dof_, std::back_inserter(states->position));
        std::copy(x.begin() + dof_, x.begin() + 2 * dof_, std::back_inserter(states->velocity));
        states->effort = command_->at("torque");
        auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        states->header.stamp = rclcpp::Time(time.count());
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
        // wait for real time elapse
        if (is_sim_real_time_)
        {
            auto until = sim_start_time_ + std::chrono::duration<double>(t);
            std::this_thread::sleep_until(until);
        }
    }

    bool ControlManager::is_simulation()
    {
        return is_simulation_;
    }

    void ControlManager::start_simulation(double time)
    {
        typedef std::vector<double> state_type;
        auto dynamics = std::bind(&ControlManager::robot_dynamics, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2,
                                  std::placeholders::_3);

        auto observer = std::bind(&ControlManager::simulation_observer, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2);

        // Error stepper, used to create the controlled stepper
        typedef runge_kutta_cash_karp54<state_type> error_stepper_type;
        typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
        state_type x0{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        sim_start_time_ = std::chrono::steady_clock::now();
        integrate_adaptive(make_controlled(1.0e-9, 1.0e-5, error_stepper_type()), dynamics, x0, 0.0, 10.0, 0.01, observer);
        // size_t steps = integrate_adaptive(runge_kutta4<std::vector<double>>(), dynamics, x0, 0.0, time, 0.01, observer);
    }

    std::vector<double> ControlManager::simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext)
    {
        int n = dof_;
        std::vector<double> f{fext(0, n - 1), fext(1, n - 1), fext(2, n - 1),
                              fext(3, n - 1), fext(4, n - 1), fext(5, n - 1)};
        hardware_->write_state(x, f);
        controller_->update(robot_, state_, command_);
        return (*command_)["torque"];
    }

}
