#include "robot_controller_interface/controller_interface.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include <iostream>

using namespace std;
namespace controllers
{
    class Med7Controller : public controller_interface::ControllerInterface
    {
    public:
        Med7Controller() {}
        ~Med7Controller()
        {
            if (data_logger_)
                data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/med7/", "med7");
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            time_ = 0;
            timestamp_ = 0;
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(timestamp_),
                },
                std::initializer_list<ExperimentContext>{
                },
                1000);
            return CallbackReturn::SUCCESS;
        }
        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            time_ += period.seconds();
            const std::vector<double> &q_init_vec = state_->get<double>("initial_position");
            std::vector<double> &q_cmd_vec = command_->get<double>("position");
            if (!state_->get<bool>("active")[0])
            {
                q_cmd_vec = q_init_vec;
                timestamp_ = time_;
                data_logger_->record();
                return;
            }
            const std::vector<double> &q_ipo_vec = state_->get<double>("ipo_position");
            q_cmd_vec = q_ipo_vec;
            if (ijp_.empty())
            {
                ijp_ = q_ipo_vec;
                cout << "ijp_:";
                for (int i = 0; i < q_ipo_vec.size(); i++)
                cout << q_ipo_vec[i] << " ";
                cout << endl;
            }
            // 7 joint sine wave
            q_cmd_vec = ijp_;
            double pos = ijp_[6] + 0.5 * sin(0.5 * M_PI * (time_ - timestamp_));
            q_cmd_vec[6] = pos;
            data_logger_->record();
        }
       
    protected:
        vector<double> ijp_;
        double time_, timestamp_;
        std::unique_ptr<DataLogger> data_logger_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::Med7Controller, controller_interface::ControllerInterface)