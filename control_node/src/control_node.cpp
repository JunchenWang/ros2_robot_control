#include <chrono>
#include <errno.h>
#include <memory>
#include <string>
#include <thread>

#include "control_node/control_manager.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int kSchedPriority = 50;

    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::string manager_node_name = "control_node";

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);

    std::vector<std::string> node_arguments = node_options.arguments();
    for (int i = 1; i < argc; ++i)
    {
        if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
        {
            // A simple way to reject non ros args
            continue;
        }
        node_arguments.push_back(argv[i]);
    }
    node_options.arguments(node_arguments);

    auto cm = std::make_shared<control_node::ControlManager>(
        executor, manager_node_name, "", node_options);

    // /etc/security/limits.conf --> @realtime soft memlock 102400000 or, the bad_alloc exception occurs
    const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", true);
    std::string message;
    if (lock_memory && !realtime_tools::lock_memory(message))
    {
        RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory : '%s'", message.c_str());
    }

    const int cpu_affinity = cm->get_parameter_or<int>("cpu_affinity", -1);
    if (cpu_affinity >= 0)
    {
        const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity);
        if (!affinity_result.first)
        {
            RCLCPP_WARN(
                cm->get_logger(), "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str());
        }
    }

    RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
    const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
    RCLCPP_INFO(
        cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
        thread_priority);

    std::thread cm_thread(
        [cm, thread_priority]()
        {
            if (!realtime_tools::configure_sched_fifo(thread_priority))
            {
                RCLCPP_WARN(
                    cm->get_logger(),
                    "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
                    "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
                    "for details on how to enable realtime scheduling.",
                    errno, strerror(errno));
            }
            else
            {
                RCLCPP_INFO(
                    cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
                    thread_priority);
            }

            while (rclcpp::ok())
            {
                cm->prepare_loop();
                if (cm->is_simulation())
                    cm->start_simulation(10);
                else
                    cm->control_loop();
                cm->end_loop();
            }
            cm->shutdown_robot();
        });
    executor->add_node(cm);
    executor->spin();
    cm_thread.join();
    rclcpp::shutdown();
    return 0;
}
