#include "hardware_interface/sensor_interface.hpp"

using namespace robot_math;

namespace hardware_interface
{

    SensorInterface::SensorInterface() : is_running_(false)
    {
    }
    SensorInterface::~SensorInterface()
    {
        if (thread_ && thread_->joinable())
        {
            is_running_ = false;
            thread_->join();
        }
        thread_ = nullptr;
    }

    void SensorInterface::read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/)
    {
        state_[state_name_] = *real_time_buffer_.readFromRT();
    }

    CallbackReturn SensorInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int len = 0;
        node_->get_parameter_or<std::string>("state_interface", state_name_, "");
        node_->get_parameter_or<int>("length", len, 0);
        if(!state_name_.empty() && len > 0)
        {
            state_[state_name_] = std::vector<double>(len, 0);
            real_time_buffer_.initRT(state_[state_name_]);
            return CallbackReturn::SUCCESS;
        }
        return CallbackReturn::FAILURE;
    }

    // void SensorInterface::compensate_gravity(const std::vector<double> &pose)
    // {
    //     Eigen::Matrix4d T = pose_to_tform(pose) * pose_to_tform(pose_);
    //     Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();

    //     Eigen::Vector3d g = R * Eigen::Vector3d(0, 0, -1) * mass_;
    //     Eigen::Vector3d M = Eigen::Vector3d(cog_[0], cog_[1], cog_[2]).cross(g);
    //     auto & force = state_["force"];
    //     force[0] -= g(0) + offset_[0];
    //     force[1] -= g(1) + offset_[1];
    //     force[2] -= g(2) + offset_[2];
    //     force[3] -= M(0) + offset_[3];
    //     force[4] -= M(1) + offset_[4];
    //     force[5] -= M(2) + offset_[5]; 
    // }
}
