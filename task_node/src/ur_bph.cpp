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
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <fstream>

/**
 * @file ur_bph.cpp
 * @brief 基于速度级逆运动学的 UR5e 远心运动（RCM）控制实验。
 *
 * 控制器把 6 个机器人关节和工具轴上的插值系数 lambda 组成 7 维广义变量：
 *
 *     y = [q1, q2, ..., q6, lambda]^T
 *
 * P1、P2 是工具坐标系中的两个固定点，二者定义工具轴；RCM 点定义为
 *
 *     p_rcm = p1 + lambda * (p2 - p1),  lambda in [0, 1].
 *
 * 控制任务由两部分组成：
 *   1. 保持 p_rcm 在启动时的位置不动（3 维）；
 *   2. 让工具点 p2 跟踪给定轨迹（3 维）。
 *
 * 因而任务雅可比 J 为 6x7。代码通过阻尼最小二乘广义逆计算广义速度，并
 * 通过零空间项使关节趋向启动姿态。随后对 7 维广义速度进行统一缩放：既
 * 限制关节速度不超过 2 rad/s，也保证积分后的 lambda 留在 [0, 1]。最终只
 * 把前 6 维关节速度通过 RTDE speedJ 下发，第 7 维速度用于积分 lambda。
 *
 * 注意：本文件会直接向真实机器人发送速度命令，运行前必须确认机器人 IP、
 * TCP 几何尺寸、工作空间及急停条件均正确。
 */
using namespace std::chrono_literals;
using namespace robot_math;

// 预留的误差日志接口；当前实现位于文件末尾且被注释，控制流程未调用。
void log_error_to_txt(const std::vector<double> &error, const std::string &file_path);
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 控制运算在线程中执行；尝试使用优先级 50 的 SCHED_FIFO 实时调度策略。
    int kSchedPriority = 50;
    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("ur_bph", "", node_options);

    // 锁定当前进程的内存页，减少控制过程中缺页导致的时延抖动。
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
            // RTDE 直接连接真实机器人；该地址目前是硬编码值。
            auto robot_ip = "192.168.110.46";
            // auto robot_ip = "192.168.1.50";
            auto control_interface = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip);
            auto receive_interface = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip);

            // 从 launch 文件传入 URDF，并转换成 robot_math 使用的运动学模型。
            auto robot_description = node->get_parameter_or<std::string>("robot_description", "");
            std::vector<std::string> jt_names;
            std::string base, fl;
            auto robot = urdf_to_robot(robot_description, jt_names, fl, base);

            // RTDE 控制周期为 2 ms（500 Hz）。initPeriod()/waitPeriod() 负责周期同步。
            bool first_loop = true;
            int update_rate = 500;
            auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate);
            rclcpp::Time previous_time;
            rclcpp::Duration measured_period(0, 0);
            double total_time = 0;

            // 工具坐标系中的轴线端点：P1 为工具原点，P2 沿工具 z 轴偏移 0.312 m。
            // 小写 p1、p2 表示对应点变换到机器人基坐标系后的坐标。
            auto P1 = Eigen::Vector3d(0, 0, 0);
            auto P2 = Eigen::Vector3d(0, 0, 0.312);
            Eigen::Matrix4d T1, T2, invT1, invT2;
            // J1/J2：点 p1/p2 的线速度雅可比（3x6）。
            Eigen::Matrix<double, 3, 6> J1, J2;
            // Jrcm 对 [q; lambda] 求导，因此比普通关节雅可比多一列（3x7）。
            Eigen::Matrix<double, 3, 7> Jrcm;
            // 增广任务雅可比：上 3 行约束 RCM 点，下 3 行控制工具点 p2。
            Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();
            Eigen::Matrix6d JJt;
            Eigen::Matrix7d N;
            Eigen::Matrix7d I = Eigen::Matrix7d::Identity();
            T1 << Eigen::Matrix3d::Identity(), P1,
                0, 0, 0, 1;
            T2 << Eigen::Matrix3d::Identity(), P2,
                0, 0, 0, 1;

            // invT1/invT2 把末端体雅可比转换到工具上的两个偏置点。
            invT1 = inv_tform(T1);
            invT2 = inv_tform(T2);

            // 记录启动关节角，后续作为零空间姿态目标。
            auto q = receive_interface->getActualQ();
            Eigen::Vector6d init_q(q[0], q[1], q[2], q[3], q[4], q[5]);
            Eigen::Matrix4d T;
            forward_kinematics(&robot, q, T);
            auto R = T.block<3, 3>(0, 0);
            auto p = T.block<3, 1>(0, 3);
            Eigen::Vector3d p1 = R * P1 + p;
            Eigen::Vector3d p2 = R * P2 + p;
            Eigen::Vector3d prcm_act;

            // lambda 是 RCM 点在线段 p1--p2 上的位置比例。
            auto lambda = 0.7;
            // 启动时的 RCM 坐标作为整个实验过程中的固定目标。
            Eigen::Vector3d prcm = p1 + lambda * (p2 - p1);

            // 保存 p2 的初始位置，并建立垂直于初始工具轴的轨迹平面基 {r0, up0}。
            auto p2_0 = p2;
            Eigen::Vector3d dir0 = (p2 - p1).normalized();
            Eigen::Vector3d reference =
                std::abs(dir0.dot(Eigen::Vector3d::UnitZ())) < 0.9
                    ? Eigen::Vector3d::UnitZ()
                    : Eigen::Vector3d::UnitX();

            Eigen::Vector3d r0 = dir0.cross(reference).normalized();
            Eigen::Vector3d up0 = dir0.cross(r0).normalized();

            // RCM 位置误差和 p2 位置误差分别使用相同的比例增益。
            Eigen::Vector6d kp_pos(30, 30, 30, 30, 30, 30);
            Eigen::MatrixXd Jb;
            // omega 为 p2 圆周运动角速度；radius 按低频正弦包络变化。
            // pitch/pi 当前仅用于被注释的螺旋线轨迹表达式。
            auto pitch = 0.01, omega = 1.0, radius = 0.0, pi = std::acos(-1);
            Eigen::Vector6d xe, v, qd_cmd;
            Eigen::Vector7d w;
            std::vector<double> error;
            error.reserve(2 * 100000);
            auto dt = std::chrono::duration<double>(period).count();
            unsigned long long k = 0;
            // auto next_iteration_time = std::chrono::steady_clock::now();
            while (rclcpp::ok())
            {
                // 记录本周期起点，waitPeriod() 会等待至 RTDE 周期结束。
                auto t_start = control_interface->initPeriod();
                // auto current_time = node->now();
                // if (first_loop)
                //     first_loop = false;
                // else
                //     measured_period = current_time - previous_time, total_time += measured_period.seconds();
                // previous_time = current_time;

                // execute update loop
                // RCLCPP_INFO(node->get_logger(), "%f sec.", measured_period.seconds());
                q = receive_interface->getActualQ();
                // Jb 为当前末端的体雅可比，T 为工具坐标系相对基坐标系的位姿。
                jacobian_matrix(&robot, q, Jb, T);

                R = T.block<3, 3>(0, 0);
                p = T.block<3, 1>(0, 3);
                p1 = R * P1 + p;
                p2 = R * P2 + p;
                // 将末端体雅可比平移到 P1/P2，再旋转到基坐标系，取线速度部分。
                J1 = R * (adjoint_T(invT1) * Jb).bottomRows(3);
                J2 = R * (adjoint_T(invT2) * Jb).bottomRows(3);

                // p_rcm = p1 + lambda*(p2-p1)，故：
                // d(p_rcm)/dt = [J1 + lambda*(J2-J1), p2-p1] * [dq; d_lambda].
                Jrcm << J1 + lambda * (J2 - J1), p2 - p1;
                J.block<3, 7>(0, 0) = Jrcm;
                // p2 与 lambda 无关，因此第 7 列保持初始化时的零值。
                J.block<3, 6>(3, 0) = J2;
                // J << Jrcm, J2, Eigen::Vector3d::Zero();
                prcm_act = p1 + lambda * (p2 - p1);

                // p2 目标在垂直于初始工具轴的平面内运动。轨迹半径以 0.04 m
                // 为幅值、按 0.01 rad/s 的正弦包络变化，圆周相位按 omega 变化。
                radius = 0.04 * std::sin(0.01 * total_time);
                auto p2_d = p2_0 + std::cos(omega * total_time) * radius * r0 + std::sin(omega * total_time) * radius * up0; // + pitch * omega / (2 * pi) * total_time * dir0;
                // 误差向量：[固定 RCM 点误差; p2 轨迹跟踪误差]。
                xe << prcm - prcm_act, p2_d - p2;
                // RCM 偏离超过 1 cm 时退出速度控制循环。
                if (xe.topRows(3).norm() > 0.01)
                {
                    std::cout << xe << std::endl;
                    break;
                }

                error.push_back(xe.topRows(3).norm());
                error.push_back(xe.bottomRows(3).norm());
                // std::cout << xe.topRows(3).norm() << " " << xe.bottomRows(3).norm() << std::endl;
                // 任务空间比例控制：期望任务速度 v = Kp * xe。
                v = (kp_pos.array() * xe.array()).matrix();
                // 零空间参考速度只驱动 6 个关节回到启动姿态，不直接驱动 lambda。
                w << init_q - Eigen::Map<Eigen::Vector6d>(&q[0]), 0;
                // 阻尼最小二乘广义逆：J# = J^T (J J^T + damping^2 I)^-1。
                // 正则项可降低雅可比接近奇异时的速度放大。
                constexpr double damping = 1e-2;
                Eigen::Matrix6d regularized =
                    J * J.transpose() +
                    damping * damping * Eigen::Matrix6d::Identity();

                Eigen::LDLT<Eigen::Matrix6d> solver(regularized);
                Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * solver.solve(Eigen::Matrix6d::Identity());
                // N 将姿态恢复速度投影到任务的近似零空间；阻尼存在时并非精确投影。
                N = I - J_pinv * J;
                Eigen::Vector7d v_d = J_pinv * v + N * w;

                // 对整个广义速度向量使用同一个缩放系数，以保持各分量方向比例。
                // 第一项约束 6 个关节速度的绝对值不超过 2 rad/s。
                double scale = 1.0;

                for (int i = 0; i < 6; ++i)
                {
                    if (std::abs(v_d(i)) > 2.0)
                        scale = std::min(scale, 2.0 / std::abs(v_d(i)));
                }

                // 第二项按本周期可用距离限制 d_lambda，防止积分越过 [0, 1]。
                // if (v_d(6) > 0.0)
                //     scale = std::min(scale, (1.0 - lambda) / (v_d(6) * dt));
                // else if (v_d(6) < 0.0)
                //     scale = std::min(scale, -lambda / (v_d(6) * dt));

                v_d *= scale;
                // 使用缩放后的第 7 维广义速度更新 RCM 插值系数。
                lambda += v_d(6) * dt;
                qd_cmd = v_d.topRows(6);
                std::vector<double> cmd{qd_cmd(0), qd_cmd(1), qd_cmd(2), qd_cmd(3), qd_cmd(4), qd_cmd(5)};
                if (k % 200 == 0)
                    RCLCPP_INFO(node->get_logger(), "%f and %f, %f,(%f, %f, %f, %f, %f, %f)", xe.topRows(3).norm(), xe.bottomRows(3).norm(),
                                lambda, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);

                // acceleration=1 rad/s^2，time=dt；实际周期由 waitPeriod() 保持。
                control_interface->speedJ(cmd, 1, dt);
                total_time += dt;
                k++;
                control_interface->waitPeriod(t_start);

                // wait until we hit the end of the period
                // next_iteration_time += period;
                // const auto steady_now = std::chrono::steady_clock::now();
                // if (steady_now < next_iteration_time)
                // {
                //     std::this_thread::sleep_until(next_iteration_time);
                // }
                // else
                // {
                //     // The loop is late. Reset the schedule to avoid accumulating delay.
                //     next_iteration_time = steady_now;
                // }
            }
            // 控制循环结束后输出两类误差的历史最大值，并停止 UR 控制脚本。
            if (error.size() > 0)
            {
                using ErrorMatrix =
                    Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

                Eigen::Map<ErrorMatrix> err(error.data(), error.size() / 2, 2);
                std::cout << err.colwise().maxCoeff() << std::endl;
            }

            control_interface->speedStop();
            control_interface->stopScript();

            // log_error_to_txt(error,"error_log.txt");
        });
    executor->add_node(node);
    executor->spin();
    thread->join();
    rclcpp::shutdown();
    return 0;
};

// void log_error_to_txt(const std::vector<double>&error,const std::string& file_path)
// {
//     std::ofstream ofs(file_path,std::ios::app);
//     if(!ofs.is_open())
//     {
//         std::cerr<<"failed to open file:"<<file_path<<std::endl;
//         return;
//     }
//     for(int i=0;i<error.row();++i)
//     {
//        for(int j=0;j<error.cols();++j)
//        {

//        }
//     }
//     ofs<<'\n';
// }
