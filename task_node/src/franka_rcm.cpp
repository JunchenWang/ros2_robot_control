#include "DataComm.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include <atomic>
#include <chrono>
#include <errno.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <memory>
#include <string>
#include <thread>
// this is a template
using namespace std::chrono_literals;
using namespace robot_math;

Eigen::VectorXd saturate_torque(const Eigen::VectorXd &tau_d_calculated, const Eigen::VectorXd &tau_J_d)
{
    Eigen::VectorXd tau_d_saturated(7);
    for (int i = 0; i < 7; i++)
    {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, 1.0), -1.0);
    }
    return tau_d_saturated;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    DataComm::getInstance()->setDestAddress("127.0.0.1", 7755);
    int kSchedPriority = 50;
    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto node = std::make_shared<rclcpp::Node>("task_node");
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

            // Robot myrobot = load_robot("/home/wjc/panda_correct.json");
            Robot myrobot = load_robot("/home/wjc/fc3.json");
            try
            {
                RobotData robotData{0};
                franka::Robot robot("192.168.1.101");
                robot.setLoad(0.3, {0, 0, 0.02}, {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6});
                robot.automaticErrorRecovery();
                robot.setCollisionBehavior(
                    {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                    {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
                robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 200, 200}});
                robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
                franka::Model model(robot.loadModel());
                std::array<double, 7> Tau_c_array = {0, 0, 0, 0, 0, 0, 0};
                double t = 0;
                MovingFilter<double> qdFilter(7, 50);
                MovingFilter<double> VFilter(6, 50);
                MovingFilter<double> taod_Filter(7, 50);
                MovingFilter<double> taoc_Filter(7, 50);
                Eigen::Vector3d Kx(3000, 3000, 3000);
                Eigen::Vector3d Bx(90, 90, 90);
                Eigen::Vector7d Kn = 0 * Eigen::Vector7d::Ones();
                Eigen::Vector7d Bn = 1 * Eigen::Vector7d::Ones();
                // Eigen::Vector7d Y(8, 8, 8, 0.5, 8, 8, 8);
                Eigen::Vector7d Y = 3 * Eigen::Vector7d::Ones();
                // Y.tail(3) << 0, 0, 0;
                Eigen::Vector7d z = Eigen::Vector7d::Zero();
                Eigen::Vector7d dq;
                // double lamda = 0.8;
                Eigen::Vector3d p1_F(0, 0, 0), p2_F(0, 0, 0.215);
                Eigen::Vector3d x_d;
                // myrobot.ME[14] += 0.2;
                // myrobot.mass[6] += 0.2319;//end  effector
                // myrobot.mass[6] += 0.25;

                robot.control(
                    [&robotData, &myrobot, &model, &t, &Tau_c_array, &taoc_Filter, &taod_Filter, &Kx, &Bx, &Kn, &Bn, &Y, &z, &dq, &x_d, &p1_F, &p2_F](const franka::RobotState &state, franka::Duration period) -> franka::Torques
                    {
                        t += period.toSec();

                        Eigen::Map<const Eigen::Vector7d> dq(state.dq.data());
                        // qdFilter.filtering(state.dq.data(), dq.data());
                        Eigen::Map<const Eigen::Vector7d> q(state.q.data());
                        Eigen::Map<const Eigen::Vector7d> tau_ext(state.tau_ext_hat_filtered.data());
                        // Eigen::Map<const Eigen::Vector7d> q_d(state.q_d.data());
                        // Eigen::Map<const Eigen::Vector7d> dq_d(state.dq_d.data());
                        // Eigen::Map<const Eigen::Vector7d> ddq_d(state.ddq_d.data());
                        // Eigen::Map<const Eigen::Vector7d> ddq(state.ddq_d.data());
                        std::array<double, 7> g_array = model.gravity(state);
                        Eigen::Map<const Eigen::Vector7d> G(g_array.data());

                        std::array<double, 7> c_array = model.coriolis(state);
                        Eigen::Map<const Eigen::Vector7d> c(c_array.data());

                        Eigen::MatrixXd M, C, Jb, dJb, dM;
                        Eigen::Matrix4d dT, T;
                        Eigen::VectorXd g;

                        m_c_g_matrix(&myrobot, std::vector<double>(state.q.begin(), state.q.end()), std::vector<double>(state.dq.begin(), state.dq.end()),
                                     M, C, g, Jb, dJb, dM, dT, T);
                        // std::cout << cond_Matrix(Jb * Jb.transpose())(0) << "\n";
                        Eigen::Vector3d p1 = T.block<3, 3>(0, 0) * p1_F + T.block<3, 1>(0, 3);
                        Eigen::Vector3d p2 = T.block<3, 3>(0, 0) * p2_F + T.block<3, 1>(0, 3);
                        Eigen::Vector7d P = (Y.array() * dq.array()).matrix();
                        if (t == 0)
                        {
                            x_d = p1 + 0.67442 * (p2 - p1);
                            z = -P;
                        }

                        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(state.tau_J_d.data());
                        Eigen::Map<Eigen::Vector7d> Tau_c(Tau_c_array.data());

                        Eigen::LDLT<Eigen::MatrixXd> ldlt(M);

                        Eigen::Vector3d a = x_d - p1, b = p2 - p1, x;
                        double lamda = (a.dot(b) / b.dot(b));

                        Eigen::MatrixXd J_rcm, dJ_rcm;
                        std::vector<double> q_vec = {q[0], q[1], q[2], q[3], q[4], q[5], q[6]};
                        std::vector<double> dq_vec = {dq[0], dq[1], dq[2], dq[3], dq[4], dq[5], dq[6]};
                        rcm_Jacobian(&myrobot, q_vec, dq_vec, p1_F, p2_F, x_d, Jb, dJb, T, dT, J_rcm, dJ_rcm, x);

                        // std::cout << singular_values << "\n\n";
                        // std::cout << cond << "\n\n";

                        Eigen::Vector3d xe = x_d - x;
                        Eigen::Vector7d dqe = -dq;
                        Eigen::Vector3d dxe = -J_rcm * dq;
                        Eigen::Vector3d ddx_c = A_x_inv(J_rcm, M) * (Mu_x_X(J_rcm, M, dJ_rcm, C, dxe) - (Bx.array() * (J_rcm * dq).array()).matrix() + (Kx.array() * xe.array()).matrix());
                        Eigen::Vector7d tau_x = M * J_sharp_X(J_rcm, M, ddx_c - dJ_rcm * dq);
                        Eigen::Vector7d tau_n = M * null_proj(J_rcm, M, ldlt.solve((Bn.array() * dqe.array()).matrix()));

                        Eigen::Vector7d tao_d = z + P;
                        Eigen::Vector7d disturb = tao_d;
                        // std::cerr << period.toSec() << std::endl;
                        tao_d = M * J_sharp_X(J_rcm, M, J_rcm * ldlt.solve(tao_d));
                        taod_Filter.filtering(tao_d.data(), tao_d.data());
                        Tau_c = tau_x + tau_n + c - tao_d;
                        // Tau_c = tau_x + tau_n + C * dq - G + g;
                        Eigen::Vector7d tem = c - Tau_c - P - z;
                        // update z
                        Eigen::MatrixXd pinvM = pinv(M, 1e-3);
                        z += (Y.array() * (pinvM * tem).array()).matrix() * period.toSec();

                        // taoc_Filter.filtering(Tau_c.data(), Tau_c.data());
                        Tau_c = saturate_torque(Tau_c, tau_J_d);
                        // std::cerr << xe.norm() << std::endl;
                        log2Channel(robotData, 0, xe.data(), 3);
                        log2Channel(robotData, 1, dq.data(), 7);
                        log2Channel(robotData, 2, disturb.data(), 7);
                        log2Channel(robotData, 3, tau_ext.data(), 7);
                        // std::cout << lamda << "\n\n";
                        //  std::cout << dq.norm() << " " << lamda << "\n\n";
                        robotData.t = t;
                        DataComm::getInstance()->sendRobotStatus(robotData);
                        // std::cout << lamda << "\n\n";
                        //  std::cout << dq.norm() << " " << lamda << "\n\n";
                        return Tau_c_array;
                    });
            }
            catch (const franka::Exception &e)
            {
                std::cout << e.what() << std::endl;
                return nullptr;
            }
            catch (const std::exception &e)
            {
                std::cout << e.what() << std::endl;
                return nullptr;
            }
        });
    executor->add_node(node);
    executor->spin();
    thread->join();
    rclcpp::shutdown();
    return 0;
}
