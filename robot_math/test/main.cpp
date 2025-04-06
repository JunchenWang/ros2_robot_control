#include "robot_math/robot_math.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include "robot_math/ScaleFunction.hpp"
#include "robot_math/CartesianTrajectoryPlanner.hpp"
#include "robot_math/trajectory.hpp"
using namespace robot_math;

int main()
{
    // Eigen::Vector3d r(0, 0, 0);
    // Eigen::Vector3d dr(0.12, 0.13, 0.32);
    // std::cout << A_r(r) << "\n";
    // std::cout << dA_r(r, dr) << "\n";
    std::vector<double> traj = {0, 1, 1, 2, 0, 0, 3.14,
                                1, 2, 2, 2, 0, 0, 3.15,
                                2, 3, 2, 2, 0, 0, 3.16,
                                3, 4, 1, 2, 0, 0, 4,
                                4, 3, 0, 2, 0, 0, 5, 
                                5, 2, 0, 2, 0, 0, 6};
    CartesianTrajectory trajectory(traj);
     std::ofstream fout("data.txt");
     Eigen::Matrix4d Td;
     Eigen::Vector6d V, dV;;
    for(int i = 0; i <= 500; i++)
    {
        double t = i * 0.01;
        trajectory.evaluate(t, Td, V, dV);
        fout << t << " " << Td(0,3) << " " << Td(1,3) << " " << V.norm() << " " << dV.norm() << std::endl;
    }
    // std::vector<double> p0 = {-0.817259, -0.232391, 0.060765, 1.578384, -0.000002, 0.001729};
    // std::vector<double> p1 = {-0.417259, -0.432391, 0.160765, 0, -0.000002, 0.001729};
    // //
    // CartesianTrajectoryPlanner planner;
    // planner.generate_speed(pose_to_tform(p0), pose_to_tform(p1), 0.5);

    // // ScaleFunction s;
    // // s.generate(5);
    // std::ofstream fout("data.txt");
    // for(int i = 0; i <= 500; i++)
    // {
    //     double t = i * 0.01;
    //     Eigen::Matrix4d T;
    //     Eigen::Vector6d V, dV;
    //     planner.evaluate(t, T, V, dV);
    //     fout << t << " " << T(0,3) << " " << T(1,3) << " " << T(2,3) << std::endl;
    // }
    return 0;
    std::ifstream fin("/home/wjc/ros2_ws/urdf/fr3.urdf");
    std::string description((std::istreambuf_iterator<char>(fin)),
                            std::istreambuf_iterator<char>());
    std::vector<std::string> joint_names;
    std::string end_effector, base;;
    Robot robot = urdf_to_robot(description, joint_names, end_effector, base);
    print_robot(robot);
    std::vector<double> q{1, 2, 3, 4, 5, 6, 7};
    std::vector<double> dq{1, 2, 3, 4, 5, 6, 7};
    std::vector<double> ddq{1, 2, 3, 4, 5, 6, 7};
    Eigen::MatrixXd J;
    Eigen::Matrix4d T;
    // coder::array<double, 3> JJ;
    //  robot_math::jacobian_matrix_all(&robot, q, JJ);
    //  robot_math::print_code_array(JJ);
    Eigen::Matrix6x7d Fext;
    Fext << 1, 0, 0, 0, 0, 0, 1,
        0, 1, 0, 0, 0, 0, 2,
        0, 0, 1, 0, 0, 0, 3,
        0, 0, 0, 1, 0, 0, 4,
        0, 0, 0, 0, 1, 0, 5,
        0, 0, 0, 0, 0, 1, 6;
    // std::cout << get_ext_torque(&robot, q, Fext) << "\n";
    //   std::cout << inverse_dynamics(&robot, q, dq, ddq, Fext) << "\n";
    Eigen::MatrixXd M, C, Jb, dJb, dM;
    Eigen::Matrix4d Tb, dTb;
    Eigen::VectorXd g;
    auto t = std::chrono::system_clock::now();
    m_c_g_matrix(&robot, q, dq, M, C, g, Jb, dJb, dM, dTb, Tb);
    // derivative_jacobian_matrix(&robot, q, dq, dJb, Jb, dTb, Tb);
    auto t2 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t).count() << std::endl;
    std::cout << "M:\n";
    std::cout << M;
    std::cout << "\nC:\n";
    std::cout << C;
    std::cout << "\ng:\n";
    std::cout << g;
    std::cout << "\ndJb:\n";
    std::cout << dJb;
    std::cout << "\nJb:\n";
    std::cout << Jb << "\n";
    std::cout << "\ndM:\n";
    std::cout << dM;
    std::cout << "\ndTb:\n";
    std::cout << dTb << "\n";
    std::cout << "\nTb:\n";
    std::cout << Tb << "\n";
    forward_kinematics(&robot, q, T);
    std::cout << "\nT:\n";
    std::cout << T << "\n";
    // std::cout << "\ndTb:\n";
    // std::cout << dTb << "\n";
    // std::cout << mass_matrix(&robot, q) << "\n";
    // std::cout << description << std::endl;
    return 0;
}