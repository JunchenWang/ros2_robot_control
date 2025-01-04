#include "hardware_interface/ft_sensor_interface.hpp"

using namespace robot_math;

namespace hardware_interface
{

    FTSensorInterface::FTSensorInterface()
    {
    }
    FTSensorInterface::~FTSensorInterface()
    {
    }
    void FTSensorInterface::read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/)
    {
        state_["force"] = *real_time_buffer_.readFromRT();
    }

    CallbackReturn FTSensorInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        real_time_buffer_.initRT({0, 0, 0, 0, 0, 0});
        state_names_.emplace_back("force");
        state_["force"] = {0, 0, 0, 0, 0, 0};
        node_->get_parameter_or<double>("mass", mass_, 0);
        node_->get_parameter_or<std::vector<double>>("cog", cog_, {0, 0, 0});
        node_->get_parameter_or<std::vector<double>>("offset", offset_, {0, 0, 0, 0, 0, 0});
        node_->get_parameter_or<std::vector<double>>("pose", pose_, {0, 0, 0, 0, 0, 0});

        return CallbackReturn::SUCCESS;
    }

    void FTSensorInterface::compensate_gravity(const std::vector<double> &pose)
    {
        Eigen::Matrix4d T = pose_to_tform(pose) * pose_to_tform(pose_);
        Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();

        Eigen::Vector3d g = R * Eigen::Vector3d(0, 0, -1) * mass_;
        Eigen::Vector3d M = Eigen::Vector3d(cog_[0], cog_[1], cog_[2]).cross(g);
        auto & force = state_["force"];
        force[0] -= g(0) + offset_[0];
        force[1] -= g(1) + offset_[1];
        force[2] -= g(2) + offset_[2];
        force[3] -= M(0) + offset_[3];
        force[4] -= M(1) + offset_[4];
        force[5] -= M(2) + offset_[5]; 
    }
}
