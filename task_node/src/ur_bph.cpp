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
using namespace robot_math;
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
            auto robot = urdf_to_robot(robot_description, jt_names, fl, base);
            bool first_loop = true;
            int update_rate = 500;
            auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate);
            rclcpp::Time previous_time;
            rclcpp::Duration measured_period(0, 0);
            double total_time = 0;

            auto P2 = Eigen::Vector3d(0, -0.3, 0.108);
            auto P1 = Eigen::Vector3d(0, 0, 0.108);
            Eigen::Matrix4d T1, T2, invT1, invT2;
            Eigen::Matrix<double, 3, 6> J1, J2;
            Eigen::Matrix<double, 3, 7> Jrcm;
            Eigen::Matrix<double, 6, 7> J;
            Eigen::Matrix6d JJt;
            Eigen::Matrix7d N;
            Eigen::Matrix7d I = Eigen::Matrix7d::Identity();
            T1 << Eigen::Matrix3d::Identity(), P1,
            0, 0, 0, 1;
            T2 << Eigen::Matrix3d::Identity(), P2,
            0, 0, 0, 1;
       
            invT1 = inv_tform(T1);
            invT2 = inv_tform(T2);
       
            auto q = std::vector(6, 0.0);
            Eigen::Vector6d init_q(q[0], q[1], q[2], q[3], q[4], q[5]);
            Eigen::Matrix4d T;
            forward_kinematics(&robot, q, T);
            auto R = T.block<3, 3>(0, 0);
            auto p = T.block<3, 1>(0, 3);
            Eigen::Vector3d p1 = R * P1 + p;
            Eigen::Vector3d p2 = R * P2 + p;
            Eigen::Vector3d prcm_act;
            auto lambda = 0.9;
            Eigen::Vector3d prcm = p1 + lambda * (p2 - p1);
            auto p2_0 = p2;
            Eigen::Vector3d dir0 = (p2 - p1).normalized();
            Eigen::Vector3d z(0, 0, 1);
            auto r0 = dir0.cross(z).normalized();
            auto up0 = r0.cross(dir0);
            Eigen::Vector6d kp_pos(60, 60, 60, 60, 60, 60);
            Eigen::MatrixXd Jb;
            auto pitch = 0.01, omega = 1.0, radius = 0.0, pi = std::acos(-1);
            Eigen::Vector6d xe, v, qd_cmd;
            Eigen::Vector7d w;
            auto dt = std::chrono::duration<double>(period).count();
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
                RCLCPP_INFO(node->get_logger(), "%f sec.", measured_period.seconds());
                q = std::vector(6, 0.0);
                jacobian_matrix(&robot, q, Jb, T);

                R = T.block<3, 3>(0, 0);
                p = T.block<3, 1>(0, 3);
                p1 = R * P1 + p;
                p2 = R * P2 + p;
                J1 = R * (adjoint_T(invT1) * Jb).bottomRows(3);
                J2 = R * (adjoint_T(invT2) * Jb).bottomRows(3);
                Jrcm << J1 + lambda * (J2 - J1), p2 - p1;
                J << Jrcm, J2;
                prcm_act =  p1 + lambda * (p2 - p1);

                radius = 0.03 * std::sin(0.05 * total_time);
                auto p2_d = p2_0 + std::cos(omega * total_time) * radius * r0 + std::sin(omega * total_time) * radius * up0 + pitch * omega / (2 * pi) * total_time * dir0;
                xe << prcm - prcm_act, p2_d - p2;
                v = (kp_pos.array() * xe.array()).matrix();
                JJt = J * J.transpose();
                N = I - J.transpose() * JJt.llt().solve(J);
                w << init_q - Eigen::Map<Eigen::Vector6d>(&q[0]), 0;
                Eigen::Vector7d v_d = J.transpose() * JJt.llt().solve(v) + N * w;
                lambda += v_d(0) * dt;
                qd_cmd = v_d.topRows(6);
                
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
