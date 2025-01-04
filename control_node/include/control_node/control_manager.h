#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control_node_parameters.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_box.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include "robot_math/robot_math.hpp"
#include <functional>
#include <chrono>
#include "hardware_interface/robot_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include <pluginlib/class_loader.hpp>
#include "control_msgs/srv/control_command.hpp"
namespace control_node
{

    class ControlManager : public rclcpp::Node
    {
    public:
        ControlManager(std::shared_ptr<rclcpp::Executor> executor, const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option);
        ~ControlManager();
        int get_update_rate();
        void wait_for_active_controller();
        void control_loop();
        void prepare_loop();
        void end_loop();
        void command_callback(const std::shared_ptr<control_msgs::srv::ControlCommand::Request> request,
                                std::shared_ptr<control_msgs::srv::ControlCommand::Response> response);
        bool activate_controller(const std::string & controller_name);
        bool deactivate_controller();
        void shutdown_robot();
        void read(const rclcpp::Time &t, const rclcpp::Duration &period);
        void update(const rclcpp::Time &t, const rclcpp::Duration &period);
        void write(const rclcpp::Time &t, const rclcpp::Duration &period);
        void start_simulation(double time = 10.0); // seconds
        std::vector<double> simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext);
        Eigen::MatrixXd simulation_external_force(double t);
        void simulation_observer(const std::vector<double> &x, double t);
        bool is_simulation();
        bool is_running();
        bool is_finished();
    protected:
        pluginlib::UniquePtr<pluginlib::ClassLoader<hardware_interface::RobotInterface>> robot_loader_;
        pluginlib::UniquePtr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> controller_loader_;
        std::shared_ptr<hardware_interface::RobotInterface> robot_;
        std::vector<controller_interface::ControllerInterface::SharedPtr> controllers_;
        std::shared_ptr<controller_interface::ControllerInterface> active_controller_;
        std::shared_ptr<rclcpp::Executor> executor_;
        int update_rate_;
        Params params_;
        std::string robot_description_;
        std::shared_ptr<ParamListener> param_listener_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> real_time_publisher_;
        rclcpp::Service<control_msgs::srv::ControlCommand>::SharedPtr service_;
        std::mutex activate_controller_mutex_;
        bool is_simulation_;
        bool is_sim_real_time_;
        bool is_publish_joint_state_;
        realtime_tools::RealtimeBox<bool> running_;
        realtime_tools::RealtimeBox<bool> finished_;
        rclcpp::Time sim_start_time_;
        bool async_mode_;
    };

}