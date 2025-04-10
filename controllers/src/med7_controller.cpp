#include "controller_interface/controller_interface.hpp"
#include <iostream>

using namespace std;
namespace controllers
{
    class Med7Controller : public controller_interface::ControllerInterface
    {
    public:
        Med7Controller() 
        {
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            const std::vector<double> &q_vec = state_->get<double>("position");
            initial_time_ = node_->now().seconds();
            initial_joint_position_ = q_vec;
            cout << "initial_joint_position_:";
            for (int i = 0; i < q_vec.size(); i++)
                cout << q_vec[i] << " ";
            cout << endl;
            return CallbackReturn::SUCCESS;
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & /*period*/) override
        {
            std::vector<double> &q_cmd_vec = command_->get<double>("position");
            const std::vector<double> &q_vec = state_->get<double>("position");
            // 7 joint sine wave
            q_cmd_vec = initial_joint_position_;
            double pos = initial_joint_position_[6] + 0.2 * sin(0.1 * M_PI * (node_->now().seconds() - initial_time_));
            cout << "pos:" << pos << endl;
            q_cmd_vec[6] = pos;
        }
       
    protected:
        vector<double> initial_joint_position_;
        double initial_time_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::Med7Controller, controller_interface::ControllerInterface)