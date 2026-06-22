#include <chrono>
#include <errno.h>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"
#include "robot_math/robot_math.hpp"
// this is a template
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int kSchedPriority = 50;
    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("ur_bph", "", node_options);
    auto ret = realtime_tools::lock_memory();
    if (!ret.first)
        RCLCPP_WARN(node->get_logger(), "Unable to lock the memory : '%s'", ret.second.c_str());

    auto thread = std::make_shared<std::thread>(
        [node, kSchedPriority]()
        {
            if (!realtime_tools::configure_sched_fifo(kSchedPriority))
            {
                RCLCPP_WARN(node->get_logger(), "Could not enable FIFO RT scheduling policy");
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
                            kSchedPriority);
            }
            // for calculating sleep time
            // double dt = 1.0 / update_rate_;
            auto robot_description = node->get_parameter_or<std::string>("robot_description", "");
            std::vector<std::string> jt_names;
            std::string base, fl;
            auto robot = robot_math::urdf_to_robot(robot_description, jt_names, fl, base);
            bool first_loop = true;
            int update_rate = 500;
            auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate);
            rclcpp::Time previous_time;
            rclcpp::Duration measured_period(0, 0);
            double total_time = 0;
            auto next_iteration_time = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                // to do your stuff
                auto current_time = node->now();
                if (first_loop)
                    first_loop = false;
                else
                    measured_period = current_time - previous_time, total_time += measured_period.seconds();
                previous_time = current_time;
                
                // execute update loop
                RCLCPP_INFO(node->get_logger(),  "%f sec.", measured_period.seconds());
                auto P2 = Eigen::Vector3d(0, -0.3, 0.108);
                auto P1 = Eigen::Vector3d(0, 0, 0.108);
                



                // wait until we hit the end of the period
                next_iteration_time += period;
                const auto steady_now = std::chrono::steady_clock::now();
                if (steady_now < next_iteration_time)
                {
                    std::this_thread::sleep_until(next_iteration_time);
                }
                else
                {
                    // The loop is late. Reset the schedule to avoid accumulating delay.
                    next_iteration_time = steady_now;
                }
            }
        });
    executor->add_node(node);
    executor->spin();
    thread->join();
    rclcpp::shutdown();
    return 0;
}
