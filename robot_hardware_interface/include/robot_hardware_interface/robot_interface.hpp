#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "pluginlib/class_loader.hpp"
#include "robot_hardware_interface/command_interface.hpp"
#include "robot_hardware_interface/sensor_interface.hpp"
#include "robot_hardware_interface/state_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
namespace hardware_interface
{

    class RobotInterface : public HardwareInterface
    {
    public:
        using SuperClass = HardwareInterface;
        ~RobotInterface() {}
        RobotInterface();
        int configure_urdf(const std::string &robot_description);
        const std::vector<std::string> &get_joint_names() const { return joint_names_; }
        int get_dof() const { return dof_; }
        const urdf::Model &get_urdf_model() const { return robot_model_; }
        virtual std::vector<double> inverse_kinematics(const std::vector<double> &q, const Eigen::Matrix4d &Td);
        virtual bool is_stop() {return false;};
        void set_update_rate(int rate) { update_rate_ = rate; }
        std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> get_all_nodes();
        void robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t,
                            std::function<Eigen::MatrixXd(double)> f_external,
                            std::function<std::vector<double>(double, const std::vector<double> &, const Eigen::MatrixXd &)> controller);
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override;
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override;
        // for simulation only
        void write_state(const std::vector<double> &state, const std::vector<double> &/*force*/)
        {
            std::copy(state.begin(), state.begin() + dof_, state_.get<double>("position").begin());
            std::copy(state.begin() + dof_, state.begin() + 2 * dof_, state_.get<double>("velocity").begin());
            //components_["ft_sensor"]->write_state("force", force);
            //state_["force"] = force;
        }
        std::map<std::string, hardware_interface::CommandInterface*> &
        get_com_command_interface() 
        { 
            return com_command_; 
        }
        const std::map<std::string, const hardware_interface::StateInterface*> &
        get_com_state_interface()  const
        { 
            return com_state_; 
        }
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        const robot_math::Robot &get_robot_math()  const { return robot_;}
       
    protected:
        std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SensorInterface>> sensor_loader_;
        std::vector<std::string> joint_names_;
        std::string end_effector_;
        int dof_;
        urdf::Model robot_model_;
        robot_math::Robot robot_;
        std::map<std::string, const hardware_interface::StateInterface*> com_state_;
        std::map<std::string, hardware_interface::CommandInterface*> com_command_;
        std::map<std::string, hardware_interface::HardwareInterface::SharedPtr> components_;
        int update_rate_;
        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
    };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP