#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/hardware_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_math/robot_math.hpp"
#include "urdf/model.h"
#include <functional>
#include "geometry_msgs/msg/wrench.hpp"
#include "pluginlib/class_loader.hpp"
#include "realtime_tools/realtime_buffer.h"
namespace hardware_interface
{

    class RobotInterface : public HardwareInterface
    {
    public:
        ~RobotInterface() {}
        RobotInterface();
        int configure_urdf(const std::string &robot_description);
        const std::vector<std::string> &get_joint_names() { return joint_names_; }
        int get_dof() { return dof_; }
        const urdf::Model &get_urdf_model() { return robot_model_; }
        const robot_math::Robot &get_robot_model() { return robot_; }
        std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> get_all_nodes();
        void robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t,
                            std::function<Eigen::MatrixXd(double)> f_external,
                            std::function<std::vector<double>(double, const std::vector<double> &, const Eigen::MatrixXd &)> controller);
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override;
        // for simulation only
        void write_state(const std::vector<double> &state, const std::vector<double> &force)
        {
            std::copy(state.begin(), state.begin() + dof_, state_["position"].begin());
            std::copy(state.begin() + dof_, state.begin() + 2 * dof_, state_["velocity"].begin());
            state_["force"] = force;
        }
        void receive_wrench(const geometry_msgs::msg::Wrench::UniquePtr &msg);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::unique_ptr<pluginlib::ClassLoader<hardware_interface::HardwareInterface>> hardware_loader_;
        std::vector<std::string> joint_names_;
        int dof_;
        urdf::Model robot_model_;
        robot_math::Robot robot_;
        std::map<std::string, hardware_interface::HardwareInterface::SharedPtr> components_;
        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_receiver_;
        realtime_tools::RealtimeBuffer<std::vector<double>> real_time_buffer_force_;

    };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP