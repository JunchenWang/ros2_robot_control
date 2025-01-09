#include "robot_math/robot_math.hpp"
#include <jsoncpp/json/json.h>
#include "matlab_code/derivative_jacobian_matrix.h"
#include "matlab_code/jacobian_matrix.h"
#include "matlab_code/inverse_kin_general.h"
#include "matlab_code/forward_kin_general.h"
#include "matlab_code/m_c_g_matrix.h"
#include <iostream>
#include <fstream>
#include "urdf/model.h"
#include "robot_math/coder_array.h"
#include <iomanip>
#include <queue>
namespace robot_math
{

    Eigen::Matrix4d make_tform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
    {
        Eigen::Matrix4d T;
        T << R, t, 0, 0, 0, 1;
        return T;
    }
    Eigen::Matrix4d pose_to_tform(const std::vector<double> &pose)
    {
        Eigen::Vector3d rv(pose[3], pose[4], pose[5]);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block(0, 3, 3, 1) << pose[0], pose[1], pose[2];
        if (rv.norm() < 1e-10)
            return T;
        Eigen::AngleAxisd angax(rv.norm(), rv.normalized());
        T.block(0, 0, 3, 3) = angax.toRotationMatrix();
        return T;
    }
    void print_robot(const Robot &robot)
    {
        std::cout << "dof:\n";
        std::cout << robot.dof << "\n";
        int dof = static_cast<int>(robot.dof);
        Eigen::Map<const Eigen::Matrix4d> ME(robot.ME);
        Eigen::Map<const Eigen::Matrix4d> TCP(robot.TCP);
        std::cout << "ME:\n";
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
                std::cout << std::setw(10) << std::fixed << std::setprecision(4) << ME(i, j);
            std::cout << "\n";
        }
        std::cout << "TCP:\n";
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
                std::cout << std::setw(10) << std::fixed << std::setprecision(4) << TCP(i, j);
            std::cout << "\n";
        }
        std::cout << "com:\n";
        for (int i = 0; i < dof; i++)
        {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.com.at(i, 0);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.com.at(i, 1);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.com.at(i, 2);
            std::cout << "\n";
        }

        std::cout << "mass:\n";
        for (int i = 0; i < dof; i++)
        {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.mass[i];
        }
        std::cout << "\n";
        std::cout << "A:\n";
        for (int i = 0; i < dof; i++)
        {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.A.at(i, 0);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.A.at(i, 1);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.A.at(i, 2);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.A.at(i, 3);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.A.at(i, 4);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.A.at(i, 5);
            std::cout << "\n";
        }

        std::cout << "inertia:\n";
        for (int k = 0; k < dof; k++)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.inertia.at(i, j, k);
                }
                std::cout << "\n";
            }
            std::cout << "\n";
        }

        std::cout << "M:\n";
        for (int k = 0; k < dof; k++)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    std::cout << std::setw(10) << std::fixed << std::setprecision(4) << robot.M.at(i, j, k) << " ";
                }
                std::cout << "\n";
            }
            std::cout << "\n";
        }
    }

    void print_code_array(const coder::array<double, 3> &array)
    {
        auto sz = array.size();
        for (int k = 0; k < sz[2]; k++)
        {
            for (int i = 0; i < sz[0]; i++)
            {
                for (int j = 0; j < sz[1]; j++)
                {
                    std::cout << std::setw(10) << std::fixed << std::setprecision(4) << array.at(i, j, k);
                }
                std::cout << "\n";
            }
            std::cout << "\n";
        }
    }

    Robot urdf_to_robot(const std::string &description, std::vector<std::string> &joint_names,  std::string &link_name)
    {
        urdf::Model urdf_model;
        Robot robot;
        urdf_model.initString(description);
        std::vector<urdf::LinkSharedPtr> bodies;
        urdf::LinkSharedPtr last_link;
        for (auto link : urdf_model.links_)
        {
            last_link = link.second;
            if (link.first == link_name)
                break;
        }
        if (link_name != last_link->name) // find the farest link as the end-effector link
        {
            std::queue<urdf::LinkSharedPtr> que;
            que.push(last_link);
            while (!que.empty())
            {
                last_link = que.front();
                que.pop();
                for (auto l : last_link->child_links)
                    que.push(l);
            }
            link_name = last_link->name;
        }
        bodies.push_back(last_link);
        while (last_link = last_link->getParent())
        {
            bodies.push_back(last_link);
        }
        std::reverse(bodies.begin(), bodies.end());
        bodies.erase(bodies.begin()); // remove base
        int n = bodies.size();
        int dof = 0;
        coder::array<double, 1U> mass;
        mass.set_size(n);
        coder::array<double, 3U> inertia;
        inertia.set_size(3, 3, n);
        coder::array<double, 2U> A;
        A.set_size(n, 6);
        coder::array<double, 3U> M;
        M.set_size(4, 4, n);
        coder::array<double, 2U> com;
        com.set_size(n, 3);

        Eigen::Map<Eigen::Matrix4d> base(robot.ME);
        Eigen::Map<Eigen::Matrix4d> TCP(robot.TCP);
        Eigen::Map<Eigen::Vector3d> gravity(robot.gravity);
        base = Eigen::Matrix4d::Identity();
        TCP = Eigen::Matrix4d::Identity();
        gravity = Eigen::Vector3d(0, 0, -9.8);
        joint_names.clear();
        for (int i = 0; i < n; i++)
        {
            urdf::Pose pose = bodies[i]->parent_joint->parent_to_joint_origin_transform;
            Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
            auto R = q.toRotationMatrix();
            auto t = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Matrix4d joint_transform = base * make_tform(R, t);
            double m = 0;
            Eigen::Matrix3d I = Eigen::Matrix3d::Zero();

            auto link_inertia = bodies[i]->inertial;
            if (link_inertia)
            {
                m = link_inertia->mass;
                pose = link_inertia->origin;
                Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
                R = q.toRotationMatrix();
                t = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
                I << link_inertia->ixx, link_inertia->ixy, link_inertia->ixz,
                    link_inertia->ixy, link_inertia->iyy, link_inertia->iyz,
                    link_inertia->ixz, link_inertia->iyz, link_inertia->izz;
            }

            int type = bodies[i]->parent_joint->type;
            if (type == urdf::Joint::FIXED)
            {
                base = joint_transform;
                if (dof > 0 && m > 0)
                {
                    Eigen::Map<Eigen::Matrix3d> inert(&inertia[(dof - 1) * 9]);
                    Eigen::Matrix4d T = joint_transform * make_tform(R, t);
                    R = T.block<3, 3>(0, 0);
                    t = T.block<3, 1>(0, 3);
                    inert += R * I * R.transpose() + m * (t.squaredNorm() * Eigen::Matrix3d::Identity() - t * t.transpose());
                    double rho = m / (mass[dof - 1] + m);
                    Eigen::Vector3d c = rho * t + (1 - rho) * Eigen::Vector3d(com.at(dof - 1, 0), com.at(dof - 1, 1), com.at(dof - 1, 2));
                    com.at(dof - 1, 0) = c[0];
                    com.at(dof - 1, 1) = c[1];
                    com.at(dof - 1, 2) = c[2];
                    mass[dof - 1] += m;
                }
            }
            else
            {
                dof++;
                joint_names.emplace_back(bodies[i]->parent_joint->name);
                Eigen::Map<Eigen::Matrix3d> inert(&inertia[(dof - 1) * 9]);
                inert = R * I * R.transpose() + m * (t.squaredNorm() * Eigen::Matrix3d::Identity() - t * t.transpose());
                mass[dof - 1] = m;
                com.at(dof - 1, 0) = pose.position.x;
                com.at(dof - 1, 1) = pose.position.y;
                com.at(dof - 1, 2) = pose.position.z;
                Eigen::Map<Eigen::Matrix4d> MM(&M[(dof - 1) * 16]);
                MM = joint_transform;
                auto axis = bodies[i]->parent_joint->axis;
                if (type == urdf::Joint::REVOLUTE)
                {
                    A.at(dof - 1, 0) = axis.x;
                    A.at(dof - 1, 1) = axis.y;
                    A.at(dof - 1, 2) = axis.z;
                    A.at(dof - 1, 3) = 0;
                    A.at(dof - 1, 4) = 0;
                    A.at(dof - 1, 5) = 0;
                }
                else
                {
                    A.at(dof - 1, 0) = 0;
                    A.at(dof - 1, 1) = 0;
                    A.at(dof - 1, 2) = 0;
                    A.at(dof - 1, 3) = axis.x;
                    A.at(dof - 1, 4) = axis.y;
                    A.at(dof - 1, 5) = axis.z;
                }
                base = Eigen::Matrix4d::Identity();
            }
        }

        robot.dof = dof;
        robot.mass.set_size(dof);
        robot.com.set_size(dof, 3);
        robot.A.set_size(dof, 6);
        robot.M.set_size(4, 4, dof);
        robot.inertia.set_size(3, 3, dof);
        for (int i = 0; i < dof; i++)
        {
            robot.mass[i] = mass[i];
            robot.com.at(i, 0) = com.at(i, 0);
            robot.com.at(i, 1) = com.at(i, 1);
            robot.com.at(i, 2) = com.at(i, 2);
            robot.A.at(i, 0) = A.at(i, 0);
            robot.A.at(i, 1) = A.at(i, 1);
            robot.A.at(i, 2) = A.at(i, 2);
            robot.A.at(i, 3) = A.at(i, 3);
            robot.A.at(i, 4) = A.at(i, 4);
            robot.A.at(i, 5) = A.at(i, 5);
        }
        std::copy(&M[0], &M[0] + dof * 16, &robot.M[0]);
        std::copy(&inertia[0], &inertia[0] + dof * 9, &robot.inertia[0]);
        return robot;
    }
    Eigen::Matrix3d exp_r(const Eigen::Vector3d &r)
    {
        if (r.norm() > std::numeric_limits<double>::epsilon())
        {
            Eigen::AngleAxisd angax(r.norm(), r.normalized());
            return angax.toRotationMatrix();
        }
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d logR(const Eigen::Matrix3d &R)
    {

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R - Eigen::Matrix3d::Identity(), Eigen::ComputeFullV);
        Eigen::Vector3d v = svd.matrixV().col(2);
        Eigen::Vector3d v_hat(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        double phi = atan2(v.dot(v_hat), R.trace() - 1);
        return phi * v;
        /*Eigen::AngleAxisd ax(R);
        return ax.angle() * ax.axis();*/
    }

    Eigen::Matrix4d exp_twist(const Eigen::Vector6d &twist)
    {
        Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
        Eigen::Vector7d Sa = normalize_twist(twist);
        Eigen::Vector6d S = Sa.head(6);
        double theta = Sa(6);
        Eigen::Matrix3d W = so_w(S.head(3));
        tform.block<3, 3>(0, 0) = exp_r(twist.head(3));
        tform.block<3, 1>(0, 3) = (Eigen::Matrix3d::Identity() * theta + (1 - std::cos(theta)) * W + (theta - std::sin(theta)) * W * W) * S.segment<3>(3);
        return tform;
    }

    Eigen::Vector6d logT(const Eigen::Matrix4d &T)
    {
        Eigen::Vector6d V = Eigen::Vector6d::Zero();
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d p = T.block<3, 1>(0, 3);
        Eigen::Vector3d w = logR(R);
        double theta = w.norm();
        if (theta < std::numeric_limits<double>::epsilon())
            V.tail(3) = p;
        else
        {
            V.head(3) = w;
            Eigen::Matrix3d W = so_w(w / theta);
            Eigen::Matrix3d W2 = W * W;
            V.tail(3) = (Eigen::Matrix3d::Identity() - theta / 2 * W + (1 - theta / (2 * std::tan(theta / 2))) * W2) * p;
        }

        return V;
    }

    std::vector<double> tform_to_pose(const Eigen::Matrix4d &T)
    {
        Eigen::Vector3d w;
        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        w = logR(R);
        return {T(0, 3), T(1, 3), T(2, 3), w[0], w[1], w[2]};
    }

    Eigen::Vector3d cond_matrix(const Eigen::MatrixXd &A)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto singular_values = svd.singularValues();
        int n = singular_values.size();
        return Eigen::Vector3d(singular_values(0) / singular_values(n - 1), singular_values(0), singular_values(1));
    }

    Eigen::MatrixXd dx_to_dq(const Eigen::MatrixXd &J, const Eigen::MatrixXd &dx, double con_threshold, double lambda)
    {
        int m = J.cols();
        int n = J.rows();
        Eigen::MatrixXd JJ_t = J * J.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJ_t, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto singular_values = svd.singularValues();
        double c = singular_values(0) / singular_values(m - 1);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        if (c > con_threshold)
        {
            return (J.transpose() * J + lambda * lambda * I).ldlt().solve(J.transpose() * dx);
        }
        else
            return J.transpose() * svd.solve(dx);
    }
    void derivative_tform_inv(const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT, Eigen::Matrix4d &dinvT, Eigen::Matrix4d &invT)
    {
        Eigen::Matrix3d dR_t = dT.block<3, 3>(0, 0).transpose();
        Eigen::Vector3d dt = dT.block<3, 1>(0, 3);
        Eigen::Matrix3d R_t = T.block<3, 3>(0, 0).transpose();
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        dinvT << dR_t, -dR_t * t - R_t * dt, 0, 0, 0, 0;
        invT << R_t, -R_t * t, 0, 0, 0, 1;
    }

    void derivative_adjoint_T(const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT, Eigen::Matrix6d &dAdT, Eigen::Matrix6d &AdT)
    {
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        Eigen::Matrix3d dR = dT.block<3, 3>(0, 0);
        Eigen::Vector3d dt = dT.block<3, 1>(0, 3);
        dAdT << dR, Eigen::Matrix3d::Zero(), so_w(dt) * R + so_w(t) * dR, dR;
        AdT << R, Eigen::Matrix3d::Zero(), so_w(t) * R, R;
    }

    Eigen::MatrixXd pinv(const Eigen::MatrixXd &matrix, double tol)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd s = svd.singularValues();
        Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(s.size());
        for (int i = 0; i < singularValuesInv.size(); ++i)
        {
            if (s(i) <= tol)
            {
                singularValuesInv(i) = 0;
            }
            else
            {
                singularValuesInv(i) = 1.0 / s(i);
            }
        }
        return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    }

    Eigen::Vector6d get_ext_force(const std::vector<double> &force,
                       double mass,
                       const std::vector<double> &offset,
                       const std::vector<double> &cog,
                       const std::vector<double> &installed_pose,
                       const Eigen::Matrix4d & robot_T)
    {
        Eigen::Matrix4d T = robot_T * pose_to_tform(installed_pose);
        Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();

        Eigen::Vector3d g = R * Eigen::Vector3d(0, 0, -1) * mass;
        Eigen::Vector3d M = Eigen::Vector3d(cog[0], cog[1], cog[2]).cross(g);
        return Eigen::Vector6d(force[3] - M(0) - offset[3],
                               force[4] - M(1) - offset[4],
                               force[5] - M(2) - offset[5],
                               force[0] - g(0) - offset[0],
                               force[1] - g(1) - offset[1],
                               force[2] - g(2) - offset[2]);
    }
    void get_ext_force(float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double> &pose)
    {
        Eigen::Matrix4d T = pose_to_tform(pose);
        Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();

        Eigen::Vector3d g = R * Eigen::Vector3d(0, 0, -1) * mass;
        Eigen::Vector3d M = Eigen::Vector3d(cog[0], cog[1], cog[2]).cross(g);
        force[0] -= static_cast<float>(g(0) + offset[0]);
        force[1] -= static_cast<float>(g(1) + offset[1]);
        force[2] -= static_cast<float>(g(2) + offset[2]);
        force[3] -= static_cast<float>(M(0) + offset[3]);
        force[4] -= static_cast<float>(M(1) + offset[4]);
        force[5] -= static_cast<float>(M(2) + offset[5]);
    }

    Eigen::Matrix3d so_w(const Eigen::Vector3d &w)
    {
        Eigen::Matrix3d S;
        S << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
        return S;
    }

    Eigen::Matrix4d se_twist(const Eigen::Vector6d &V)
    {
        Eigen::Matrix4d Tv = Eigen::Matrix4d::Zero();
        Tv.block<3, 3>(0, 0) = so_w(V.head(3));
        Tv.block<3, 1>(0, 3) = V.tail(3);
        return Tv;
    }

    Eigen::Vector7d normalize_twist(const Eigen::Vector6d &V)
    {
        Eigen::Vector7d Sa = Eigen::Vector7d::Zero();
        Eigen::Vector3d w = V.head(3);
        Eigen::Vector3d v = V.tail(3);
        if (V.norm() < std::numeric_limits<double>::epsilon())
            Sa(5) = 1;
        else if (w.norm() < std::numeric_limits<double>::epsilon())
        {
            Sa(6) = v.norm();
            Sa.segment<3>(3) = v / Sa(6);
        }
        else
        {
            Sa(6) = w.norm();
            Sa.head(6) = V / Sa(6);
        }
        return Sa;
    }

    Eigen::Matrix4d inv_tform(const Eigen::Matrix4d &T)
    {
        Eigen::Matrix4d invT = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();
        invT.block(0, 0, 3, 3) = R;
        invT.block(0, 3, 3, 1) = -R * T.block(0, 3, 3, 1);
        return invT;
    }

    Eigen::Matrix6d adjoint_T(const Eigen::Matrix4d &T)
    {
        Eigen::Matrix6d AdT = Eigen::Matrix6d::Zero();
        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        AdT.block(0, 0, 3, 3) = R;
        AdT.block(3, 3, 3, 3) = R;
        AdT.block(3, 0, 3, 3) = so_w(T.block(0, 3, 3, 1)) * R;
        return AdT;
    }

    Eigen::Matrix6d adjoint_V(const Eigen::Vector6d &V)
    {
        Eigen::Matrix6d AdV;
        Eigen::Matrix3d sk = so_w(V.topRows(3));
        AdV << sk, Eigen::Matrix3d::Zero(), so_w(V.bottomRows(3)), sk;
        return AdV;
    }

    Eigen::MatrixXd J_sharp(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M)
    {
        Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
        return tem * (J * tem).inverse();
    }

    Eigen::MatrixXd d_J_sharp(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &dM)
    {
        Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
        Eigen::MatrixXd tem = ldlt.solve(J.transpose());
        Eigen::MatrixXd dtem = -ldlt.solve(dM) * ldlt.solve(J.transpose()) + ldlt.solve(dJ.transpose());
        Eigen::MatrixXd tem2 = J * tem;
        return (dtem - tem * (tem2.ldlt().solve((dJ * tem + J * dtem)))) * tem2.inverse();
    }

    Eigen::MatrixXd d_J_sharp_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &X)
    {
        Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
        Eigen::MatrixXd tem = ldlt.solve(J.transpose());
        Eigen::MatrixXd dtem = -ldlt.solve(dM) * ldlt.solve(J.transpose()) + ldlt.solve(dJ.transpose());
        // Eigen::MatrixXd tem2 = J * tem;
        Eigen::LDLT<Eigen::MatrixXd> ldlt2(J * tem);
        return (dtem - tem * (ldlt2.solve((dJ * tem + J * dtem)))) * ldlt2.solve(X);
    }
    double c = 1000;
    double cd = 5e-2;

    Eigen::MatrixXd J_sharp_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X)
    {
        Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
        Eigen::MatrixXd tem2 = J * tem;
        Eigen::Vector3d cond = cond_matrix(tem2);

        if (cond(0) > c)
        {
            // std::cout << "cond big " << cond(0) <<  "\n";
            double alpha = cd; // std::max(cd, (cond(1) * cond(1) - c * cond(2) * cond(2)) / (c- 1));
            // std::cout <<  (cond(1) * cond(1) + alpha) / (cond(2) * cond(2) + alpha) <<  "\n";
            Eigen::MatrixXd tem3 = tem2.transpose() * tem2;
            Eigen::VectorXd d(tem2.cols());
            d.setOnes();
            tem3.diagonal() += alpha * d;
            return tem * (tem3).ldlt().solve(tem2.transpose() * X);
        }

        else
            return tem * (tem2).ldlt().solve(X);
        //    return tem * pInv(tem2) * X;
    }

    Eigen::MatrixXd J_sharp_T_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X)
    {
        Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
        Eigen::MatrixXd tem = ldlt.solve(J.transpose());
        Eigen::MatrixXd tem2 = J * tem;
        Eigen::Vector3d cond = cond_matrix(tem2);
        if (cond(0) > c)
        {
            // std::cout << "cond big " << cond(0) <<  "\n";
            double alpha = cd; // std::max(cd, (cond(1) * cond(1) - c * cond(2) * cond(2)) / (c- 1));
            // std::cout <<  (cond(1) * cond(1) + alpha) / (cond(2) * cond(2) + alpha) <<  "\n";
            Eigen::MatrixXd tem3 = tem2.transpose() * tem2;
            Eigen::VectorXd d(tem2.cols());
            d.setOnes();
            tem3.diagonal() += alpha * d;
            return (tem3).ldlt().solve(tem2.transpose() * J) * ldlt.solve(X);
        }
        else
            return (tem2).ldlt().solve(J) * ldlt.solve(X);
        // return pInv(tem2) * J * ldlt.solve(X);
    }

    Eigen::MatrixXd A_x(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M)
    {
        Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
        return (J * tem).inverse();
    }

    Eigen::MatrixXd A_x_inv(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M)
    {
        return J * M.ldlt().solve(J.transpose());
    }

    Eigen::MatrixXd A_x_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X)
    {
        Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
        return (J * tem).ldlt().solve(X);
    }

    Eigen::VectorXd null_proj(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::VectorXd &v)
    {
        return v - J_sharp_X(J, M, J * v);
    }

    Eigen::MatrixXd null_z(const Eigen::MatrixXd &J)
    {
        int m = J.rows();
        int n = J.cols();
        int r = n - m;
        Eigen::Matrix6d Jm = J.leftCols(m);  // m x m
        Eigen::MatrixXd Jr = J.rightCols(r); // m x r
        Eigen::MatrixXd I(r, r);
        I.setIdentity();
        Eigen::MatrixXd Z(n, r);
        Z.topRows(m) = -Jm.partialPivLu().solve(Jr);
        Z.bottomRows(r) = I;
        return Z;
    }

    Eigen::MatrixXd z_sharp(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M)
    {
        return (Z.transpose() * M * Z).ldlt().solve(Z.transpose() * M);
    }

    Eigen::MatrixXd Mu_x_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &C, const Eigen::MatrixXd &X)
    {
        return (J_sharp_T_X(J, M, C) - A_x_X(J, M, dJ)) * J_sharp_X(J, M, X);
    }

    Eigen::MatrixXd Mu_x(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &C)
    {
        return (J_sharp_T_X(J, M, C) - A_x_X(J, M, dJ)) * J_sharp(J, M);
    }

    Eigen::MatrixXd d_null_z(const Eigen::MatrixXd &J, const Eigen::MatrixXd &dJ)
    {
        int m = J.rows();
        int n = J.cols();
        int r = n - m;
        Eigen::Matrix6d dJm = dJ.leftCols(m);  // m x m
        Eigen::MatrixXd dJr = dJ.rightCols(r); // m x r
        Eigen::Matrix6d Jm = J.leftCols(m);    // m x m
        Eigen::MatrixXd Jr = J.rightCols(r);   // m x r

        Eigen::MatrixXd I(r, r);
        I.setZero();

        Eigen::MatrixXd dZ(n, r);
        dZ.topRows(m) = Jm.partialPivLu().solve(dJm) * Jm.partialPivLu().solve(Jr) - Jm.partialPivLu().solve(dJr);
        dZ.bottomRows(r) = I;
        return dZ;
    }

    Eigen::MatrixXd d_z_sharp(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM)
    {
        Eigen::LDLT<Eigen::MatrixXd> ldlt(Z.transpose() * M * Z);
        // Eigen::MatrixXd tem1 = Z.transpose() * M * Z;
        Eigen::MatrixXd dtem1 = dZ.transpose() * M * Z + Z.transpose() * dM * Z + Z.transpose() * M * dZ;

        Eigen::MatrixXd tem2 = Z.transpose() * M;
        Eigen::MatrixXd dtem2 = dZ.transpose() * M + Z.transpose() * dM;

        return -ldlt.solve(dtem1) * ldlt.solve(tem2) + ldlt.solve(dtem2);
    }

    Eigen::MatrixXd A_v(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M)
    {
        return Z.transpose() * M * Z;
    }

    Eigen::MatrixXd Mu_v(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &C)
    {
        return (Z.transpose() * C - A_v(Z, M) * d_z_sharp(Z, M, dZ, dM)) * Z;
    }

    Eigen::MatrixXd Mu_xv(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &C)
    {
        return (J_sharp_T_X(J, M, C) - A_x_X(J, M, dJ)) * Z;
    }

    Eigen::MatrixXd Mu_vx(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &C)
    {
        return (Z.transpose() * C - A_v(Z, M) * d_z_sharp(Z, M, dZ, dM)) * J_sharp(J, M);
    }

    Eigen::MatrixXd Mu_vx_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &C, const Eigen::MatrixXd &X)
    {
        return (Z.transpose() * C - A_v(Z, M) * d_z_sharp(Z, M, dZ, dM)) * J_sharp_X(J, M, X);
    }

    void cal_motion_error(const Eigen::Matrix4d &T, const Eigen::Matrix4d &T_d,
                          const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                          const Eigen::Vector3d &v_d, const Eigen::Vector3d &w_d,
                          const Eigen::Vector3d &a_d, const Eigen::Vector3d &alpha_d,
                          Eigen::Vector6d &xe, Eigen::Vector6d &dxe, Eigen::Vector6d &ddx_d)
    {

        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Matrix3d Rd = T_d.block<3, 3>(0, 0);
        xe.head<3>() = logR(R.transpose() * Rd);
        xe.tail<3>() = T_d.block<3, 1>(0, 3) - T.block<3, 1>(0, 3);
        dxe.head<3>() = R.transpose() * (w_d - w);
        dxe.tail<3>() = v_d - v;
        ddx_d.head<3>() = R.transpose() * (alpha_d - w.cross(w_d));
        ddx_d.tail<3>() = a_d;
    }

    void swap_order(double *buf, int n)
    {
        int offset = n / 2;
        for (int i = 0; i < offset; i++)
        {
            double tem = buf[i];
            buf[i] = buf[i + offset];
            buf[i + offset] = tem;
        }
    }

    void body_twist_to_spatial(const double _T[16], const double _Vb[6], const double _dVb[6], double _Vs[6], double _dVs[6])
    {
        Eigen::Map<const Eigen::Matrix4d> T(_T);
        Eigen::Map<const Eigen::Vector6d> Vb(_Vb);

        Eigen::Map<Eigen::Vector6d> Vs(_Vs);

        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        Eigen::Matrix4d Tsb = Eigen::Matrix4d::Identity();
        Tsb.block(0, 0, 3, 3) = R;
        Vs = adjoint_T(Tsb) * Vb;
        if (_dVb && _dVs)
        {
            Eigen::Map<const Eigen::Vector6d> dVb(_dVb);
            Eigen::Map<Eigen::Vector6d> dVs(_dVs);
            dVs.topRows(3) = R * dVb.topRows(3);
            Eigen::Vector3d w = Vs.topRows(3);
            Eigen::Vector3d v = Vs.bottomRows(3);
            dVs.bottomRows(3) = w.cross(v) + R * dVb.bottomRows(3);
        }
    }

    void spatial_twist_to_body(const double _T[16], const double _Vs[6], const double _dVs[16], double _Vb[6], double _dVb[6])
    {
        Eigen::Map<const Eigen::Matrix4d> T(_T);
        Eigen::Map<Eigen::Vector6d> Vb(_Vb);
        Eigen::Map<const Eigen::Vector6d> Vs(_Vs);
        Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();
        Eigen::Matrix4d Tbs = Eigen::Matrix4d::Identity();
        Tbs.block(0, 0, 3, 3) = R;
        Vb = adjoint_T(Tbs) * Vs;
        if (_dVs && _dVb)
        {
            Eigen::Map<Eigen::Vector6d> dVb(_dVb);
            Eigen::Map<const Eigen::Vector6d> dVs(_dVs);
            Eigen::Vector3d w = Vs.topRows(3);
            Eigen::Vector3d v = Vs.bottomRows(3);
            dVb.topRows(3) = R * dVs.topRows(3);
            dVb.bottomRows(3) = R * (dVs.bottomRows(3) - w.cross(v));
        }
    }
    // spatial twist
    Eigen::Vector6d twist_estimate(const Eigen::Matrix4d &Td, const Eigen::Matrix4d &Td_pre, double dt)
    {
        Eigen::Matrix3d Rd = Td.block(0, 0, 3, 3);
        Eigen::Matrix3d RdT = Rd.transpose();
        Eigen::Matrix3d Rd_pre = Td_pre.block(0, 0, 3, 3);
        Eigen::Matrix3d dR = (Rd - Rd_pre) / dt;
        Eigen::Matrix3d W = dR * RdT;
        Eigen::Vector3d w(W(2, 1), W(0, 2), W(1, 0));
        Eigen::Vector3d v = (Td.block(0, 3, 3, 1) - Td_pre.block(0, 3, 3, 1)) / dt;
        Eigen::Vector6d Vd;
        Vd << w, v;
        return Vd;
    }

    void inverse_kin_general(const Robot *robot, Eigen::Matrix4d Td, const std::vector<double> &qref, const double tol[2], std::vector<double> &q, double *flag)
    {
        q.resize(qref.size());
        coder::array<double, 2U> q_array;
        q_array.set(&q[0], 1, q.size());
        ::inverse_kin_general(robot, Td.data(), qref, tol, q_array, flag);
    }

    void forward_kin_general(const Robot *robot, const std::vector<double> &q, Eigen::Matrix4d &T)
    {
        ::forward_kin_general(robot, q, T.data());
    }

    class jsexception : public std::exception
    {
        virtual const char *what() const throw()
        {
            return "can not read robot json file";
        }
    } jsex;

    Robot load_robot(const char *filename)
    {
        Json::Value root;
        std::ifstream ifs;
        ifs.open(filename);

        Json::CharReaderBuilder builder;
        // builder["collectComments"] = true;
        JSONCPP_STRING errs;
        if (!parseFromStream(builder, ifs, &root, &errs))
        {
            throw jsex;
        }
        Robot robot;
        int n = root["dof"].asInt();
        robot.dof = n;
        robot.mass.set_size(n);
        robot.inertia.set_size(3, 3, n);
        robot.com.set_size(n, 3);
        robot.A.set_size(n, 6);
        robot.M.set_size(4, 4, n);
        for (int i = 0; i < n; i++)
        {
            robot.mass[i] = root["mass"][i].asDouble();
            Eigen::Map<Eigen::Matrix3d> inertia(&robot.inertia[0] + 9 * i);
            Eigen::Map<Eigen::Matrix4d> M(&robot.M[0] + 16 * i);
            inertia << root["inertia"][i][0].asDouble(), root["inertia"][i][5].asDouble(), root["inertia"][i][4].asDouble(),
                root["inertia"][i][5].asDouble(), root["inertia"][i][1].asDouble(), root["inertia"][i][3].asDouble(),
                root["inertia"][i][4].asDouble(), root["inertia"][i][3].asDouble(), root["inertia"][i][2].asDouble();
            // std::cout << inertia << std::endl;
            Eigen::Vector3d w, t;
            w << root["M"][i][0].asDouble(), root["M"][i][1].asDouble(), root["M"][i][2].asDouble();
            t << root["M"][i][3].asDouble(), root["M"][i][4].asDouble(), root["M"][i][5].asDouble();
            M << exp_r(w), t, 0, 0, 0, 1;
            // std::cout << M << std::endl;
            for (int j = 0; j < 6; j++)
                robot.A[i + n * j] = root["A"][i][j].asDouble();
            for (int j = 0; j < 3; j++)
                robot.com[i + n * j] = root["com"][i][j].asDouble();
        }

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                robot.ME[i + 4 * j] = root["ME"][i][j].asDouble();
                robot.TCP[i + 4 * j] = root["TCP"][i][j].asDouble();
            }
        robot.gravity[0] = root["gravity"][0].asDouble();
        robot.gravity[1] = root["gravity"][1].asDouble();
        robot.gravity[2] = root["gravity"][2].asDouble();

        /*Eigen::Map<Eigen::Matrix<double, 7, 6>> A(&robot.A[0]);
        Eigen::Map<Eigen::Matrix<double, 7, 3>> com(&robot.com[0]);
        Eigen::Map<Eigen::Matrix4d> ME(&robot.ME[0]);
        Eigen::Map<Eigen::Matrix4d> TCP(&robot.TCP[0]);
        Eigen::Map<Eigen::Vector3d> G(&robot.gravity[0]);
        std::cout << A << std::endl;
        std::cout << com << std::endl;
        std::cout << ME << std::endl;
        std::cout << TCP << std::endl;
        std::cout << G << std::endl;*/
        return robot;
    }

    Eigen::VectorXd get_ext_torque(const Robot *robot, const std::vector<double> &q, const Eigen::MatrixXd &fext)
    {
        int n = static_cast<int>(robot->dof);
        coder::array<double, 3> J;
        jacobian_matrix_all(robot, q, J);
        Eigen::VectorXd ext_torque = Eigen::VectorXd::Zero(7);
        for (int i = 0; i < n; i++)
            ext_torque += (Eigen::Map<Eigen::MatrixXd>(J.data() + i * 6 * n, 6, n)).transpose() * fext.col(i);
        return ext_torque;
    }

    void jacobian_matrix(const Robot *robot, const std::vector<double> &q, Eigen::MatrixXd &J, Eigen::Matrix4d &T)
    {
        int n = static_cast<int>(robot->dof);
        J.resize(6, n);
        Eigen::Map<const Eigen::Matrix4d> ME(robot->ME);
        Eigen::Map<const Eigen::Matrix4d> TCP(robot->TCP);
        T = ME * TCP;
        for (int i = n; i >= 1; i--)
        {
            Eigen::Vector6d Ai(robot->A.at(i - 1, 0), robot->A.at(i - 1, 1), robot->A.at(i - 1, 2),
                               robot->A.at(i - 1, 3), robot->A.at(i - 1, 4), robot->A.at(i - 1, 5));
            J.col(i - 1) = adjoint_T(inv_tform(T)) * Ai;
            T = Eigen::Map<const Eigen::Matrix4d>(robot->M.data() + 16 * (i - 1)) * exp_twist(Ai * q[i - 1]) * T;
        }
    }
    Eigen::Matrix6d spatial_inertia_matrix(const Eigen::Matrix3d &I, double m, const Eigen::Vector3d &com)
    {
        Eigen::Matrix6d G = Eigen::Matrix6d::Zero();
        Eigen::Matrix3d skcom = so_w(com);
        G.block<3, 3>(0, 0) = I - m * (com.squaredNorm() * Eigen::Matrix3d::Identity() - com * com.transpose() + skcom * skcom);
        G.block<3, 3>(0, 3) = m * skcom;
        G.block<3, 3>(3, 0) = -G.block<3, 3>(0, 3);
        G.block<3, 3>(3, 3) = m * Eigen::Matrix3d::Identity();
        return G;
    }

    Eigen::MatrixXd mass_matrix(const Robot *robot, const std::vector<double> &q)
    {
        int n = static_cast<int>(robot->dof);
        Eigen::MatrixXd Mq = Eigen::MatrixXd::Zero(n, n);
        coder::array<double, 3> J;
        J.set_size(6, n, n);
        std::fill(J.data(), J.data() + 6 * n * n, 0.0);
        for (int i = n - 1; i >= 0; i--)
        {
            Eigen::Matrix6d G = spatial_inertia_matrix(Eigen::Map<const Eigen::Matrix3d>(robot->inertia.data() + i * 9),
                                                       robot->mass[i],
                                                       Eigen::Vector3d(robot->com.at(i, 0), robot->com.at(i, 1), robot->com.at(i, 2)));
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            for (int j = i; j >= 0; j--)
            {
                Eigen::Vector6d Aj(robot->A.at(j, 0), robot->A.at(j, 1), robot->A.at(j, 2),
                                   robot->A.at(j, 3), robot->A.at(j, 4), robot->A.at(j, 5));
                Eigen::Map<Eigen::Vector6d>(J.data() + i * 6 * n + j * 6) = adjoint_T(T) * Aj;
                Eigen::Map<const Eigen::Matrix4d> Mj(robot->M.data() + j * 16);
                T = T * exp_twist(-Aj * q[j]) * inv_tform(Mj);
            }
            Eigen::Map<Eigen::MatrixXd> Ji(J.data() + i * 6 * n, 6, n);
            Mq += Ji.transpose() * G * Ji;
        }

        return Mq;
    }

    Eigen::VectorXd inverse_dynamics(const Robot *robot, const std::vector<double> &q,
                                     const std::vector<double> &dq, const std::vector<double> &ddq,
                                     const Eigen::MatrixXd &fext)
    {
        int n = static_cast<int>(robot->dof);

        Eigen::Map<const Eigen::Matrix4d> ME(robot->ME);
        Eigen::Map<const Eigen::Matrix4d> TCP(robot->TCP);

        Eigen::Vector6d nu0 = Eigen::Vector6d::Zero();
        Eigen::Vector6d dnu0 = Eigen::Vector6d::Zero();
        dnu0(3) = -robot->gravity[0];
        dnu0(4) = -robot->gravity[1];
        dnu0(5) = -robot->gravity[2];
        Eigen::MatrixXd nu = Eigen::MatrixXd::Zero(6, n);
        Eigen::MatrixXd dnu = Eigen::MatrixXd::Zero(6, n);
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
        Eigen::Matrix4d T;
        for (int i = 0; i < n; i++)
        {
            Eigen::Vector6d Ai(robot->A.at(i, 0), robot->A.at(i, 1), robot->A.at(i, 2),
                               robot->A.at(i, 3), robot->A.at(i, 4), robot->A.at(i, 5));

            Eigen::Map<const Eigen::Matrix4d> Mi(robot->M.data() + i * 16);

            T = exp_twist(-Ai * q[i]) * inv_tform(Mi);
            Eigen::Matrix6d Map = adjoint_T(T);
            nu.col(i) = Map * nu0 + Ai * dq[i];
            nu0 = nu.col(i);
            dnu.col(i) = Map * dnu0 + adjoint_V(nu0) * Ai * dq[i] + Ai * ddq[i];
            dnu0 = dnu.col(i);
        }
        T = Eigen::Matrix4d::Identity();
        Eigen::Vector6d F = Eigen::Vector6d::Zero();
        Eigen::Vector6d f;
        for (int i = n - 1; i >= 0; i--)
        {
            if (i == n - 1)
                f = adjoint_T(inv_tform(ME * TCP)).transpose() * fext.col(i);
            else
                f = fext.col(i);
            Eigen::Matrix6d G = spatial_inertia_matrix(Eigen::Map<const Eigen::Matrix3d>(robot->inertia.data() + i * 9),
                                                       robot->mass[i],
                                                       Eigen::Vector3d(robot->com.at(i, 0), robot->com.at(i, 1), robot->com.at(i, 2)));

            Eigen::Vector6d Ai(robot->A.at(i, 0), robot->A.at(i, 1), robot->A.at(i, 2),
                               robot->A.at(i, 3), robot->A.at(i, 4), robot->A.at(i, 5));
            Eigen::Map<const Eigen::Matrix4d> Mi(robot->M.data() + i * 16);

            F = adjoint_T(T).transpose() * F + G * dnu.col(i) - adjoint_V(nu.col(i)).transpose() * (G * nu.col(i)) - f;
            tau(i) = F.dot(Ai); // + 1*dq[i];
            T = exp_twist(-Ai * q[i]) * inv_tform(Mi);
        }
        return tau;
    }

    void jacobian_matrix_all(const Robot *robot, const std::vector<double> &q, coder::array<double, 3> &J)
    {
        int n = static_cast<int>(robot->dof);
        J.set_size(6, n, n);
        std::fill(J.data(), J.data() + 6 * n * n, 0.0);
        Eigen::Map<const Eigen::Matrix4d> ME(robot->ME);
        Eigen::Map<const Eigen::Matrix4d> TCP(robot->TCP);
        Eigen::Matrix4d T;
        for (int i = n; i >= 1; i--)
        {
            if (i == n)
                T = inv_tform(ME * TCP);
            else
                T = Eigen::Matrix4d::Identity();
            for (int j = i; j >= 1; j--)
            {
                Eigen::Vector6d Aj(robot->A.at(j - 1, 0), robot->A.at(j - 1, 1), robot->A.at(j - 1, 2),
                                   robot->A.at(j - 1, 3), robot->A.at(j - 1, 4), robot->A.at(j - 1, 5));
                Eigen::Map<const Eigen::Matrix4d> Mj(robot->M.data() + (j - 1) * 16);
                Eigen::Map<Eigen::Vector6d> Jji(J.data() + (i - 1) * 6 * n + (j - 1) * 6);
                Jji = adjoint_T(T) * Aj;
                T = T * exp_twist(-Aj * q[j - 1]) * inv_tform(Mj);
            }
        }
    }

    void m_c_g_matrix(const Robot *robot, const std::vector<double> &q, const std::vector<double> &dq,
                      Eigen::MatrixXd &M, Eigen::MatrixXd &C, Eigen::VectorXd &g,
                      Eigen::MatrixXd &Jb, Eigen::MatrixXd &dJb, Eigen::MatrixXd &dM,
                      Eigen::Matrix4d &dTb, Eigen::Matrix4d &Tb)
    {
        int n = static_cast<int>(robot->dof);
        Eigen::Map<const Eigen::Vector3d> gravity(robot->gravity);
        Eigen::Map<const Eigen::Matrix4d> ME(robot->ME);
        Eigen::Map<const Eigen::Matrix4d> TCP(robot->TCP);
        coder::array<double, 3> J, dJ, pdM, G;
        coder::array<double, 4> pdJ;
        J.set_size(6, n, n);
        std::fill(J.data(), J.data() + 6 * n * n, 0.0);
        dJ.set_size(6, n, n);
        std::fill(dJ.data(), dJ.data() + 6 * n * n, 0.0);
        pdJ.set_size(6, n, n, n);
        std::fill(pdJ.data(), pdJ.data() + 6 * n * n * n, 0.0);
        pdM.set_size(n, n, n);
        std::fill(pdM.data(), pdM.data() + n * n * n, 0.0);
        G.set_size(6, 6, n);
        std::fill(G.data(), G.data() + 6 * 6 * n, 0.0);

        C = Eigen::MatrixXd::Zero(n, n);
        g = Eigen::VectorXd::Zero(n);
        M = Eigen::MatrixXd::Zero(n, n);
        dM = Eigen::MatrixXd::Zero(n, n);
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);

        // for link i
        coder::array<double, 3> pdT;
        Eigen::Matrix6d AdT, dAdT;
        Eigen::Matrix4d tform;
        pdT.set_size(4, 4, n);
        std::fill(pdT.data(), pdT.data() + 4 * 4 * n, 0.0);

        for (int i = n - 1; i >= 0; i--)
        {
            Eigen::Map<Eigen::Matrix6d> Gi(G.data() + 6 * 6 * i);
            Eigen::Map<Eigen::MatrixXd> Ji(J.data() + 6 * n * i, 6, n);
            Eigen::Map<Eigen::MatrixXd> dJi(dJ.data() + 6 * n * i, 6, n);

            Eigen::Map<const Eigen::Matrix3d> Ii(robot->inertia.data() + i * 9);
            Eigen::Vector3d ri(robot->com.at(i, 0), robot->com.at(i, 1), robot->com.at(i, 2));
            Gi = spatial_inertia_matrix(Ii, robot->mass[i], ri);
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d dT = Eigen::Matrix4d::Zero();
            std::fill(pdT.data(), pdT.data() + 4 * 4 * n, 0.0);

            for (int j = i; j >= 0; j--)
            {
                Eigen::Vector6d Aj(robot->A.at(j, 0), robot->A.at(j, 1), robot->A.at(j, 2),
                                   robot->A.at(j, 3), robot->A.at(j, 4), robot->A.at(j, 5));
                Eigen::Matrix4d sej = se_twist(-Aj);
                Eigen::Map<const Eigen::Matrix4d> Mj(robot->M.data() + j * 16);
                Eigen::Map<Eigen::Matrix4d> pdTj(pdT.data() + j * 16);

                derivative_adjoint_T(T, dT, dAdT, AdT);
                Ji.col(j) = AdT * Aj;
                dJi.col(j) = dAdT * Aj;
                tform = exp_twist(-Aj * q[j]) * inv_tform(Mj);
                // update pdT wrt. joint j ... i
                for (int k = j + 1; k <= i; k++)
                {
                    Eigen::Map<Eigen::Matrix4d> pdTk(pdT.data() + k * 16);
                    derivative_adjoint_T(T, pdTk, dAdT, AdT);
                    Eigen::Map<Eigen::Vector6d>(pdJ.data() + k * 6 * n * n + i * 6 * n + 6 * j) = dAdT * Aj;
                    pdTk *= tform;
                }
                pdTj = T * sej * tform;
                ///////////////////////////////////////
                dT = (dT + T * sej * dq[j]) * tform;
                T = T * tform;
            }
            M += Ji.transpose() * Gi * Ji;
            dM += dJi.transpose() * Gi * Ji + Ji.transpose() * Gi * dJi;
            for (int k = 0; k <= i; k++)
            {
                Eigen::Map<const Eigen::Matrix4d> pdTk(pdT.data() + k * 16);
                Eigen::Vector3d drc = -pdTk.block<3, 3>(0, 0).transpose() * (T.block<3, 1>(0, 3) - ri) - T.block<3, 3>(0, 0).transpose() * pdTk.block<3, 1>(0, 3);
                P(i, k) = -robot->mass[i] * gravity.dot(drc);
            }
            if (i == n - 1)
            {
                Eigen::Matrix4d Offset = ME * TCP;
                Tb = inv_tform(T) * Offset;
                dTb = -inv_tform(T) * dT * inv_tform(T) * Offset;
                Eigen::Matrix6d adTb = adjoint_T(inv_tform(Offset));
                Jb = adTb * Ji;
                dJb = adTb * dJi;
            }
        }
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
            {
                Eigen::Map<Eigen::MatrixXd> pdMi(pdM.data() + i * n * n, n, n);
                Eigen::Map<const Eigen::MatrixXd> pdJji(pdJ.data() + i * 6 * n * n + j * 6 * n, 6, n);
                Eigen::Map<const Eigen::MatrixXd> Jj(J.data() + 6 * n * j, 6, n);

                Eigen::Map<const Eigen::Matrix6d> Gj(G.data() + 6 * 6 * j);
                pdMi += pdJji.transpose() * Gj * Jj + Jj.transpose() * Gj * pdJji;
                g(i) += P(j, i);
            }
        for (int k = 0; k < n; k++)
            for (int j = 0; j < n; j++)
                for (int i = 0; i < n; i++)
                    C(k, j) += 0.5 * (pdM.at(k, j, i) + pdM.at(k, i, j) - pdM.at(i, j, k)) * dq[i];
    }

    void derivative_jacobian_matrix(const Robot *robot, const std::vector<double> &q, const std::vector<double> &dq, Eigen::MatrixXd &dJ, Eigen::MatrixXd &J, Eigen::Matrix4d &dT, Eigen::Matrix4d &T)
    {
        int n = static_cast<int>(robot->dof);
        Eigen::Map<const Eigen::Matrix4d> ME(robot->ME);
        Eigen::Map<const Eigen::Matrix4d> TCP(robot->TCP);
        T = ME * TCP;
        dT.setZero();
        J.setZero(6, n);
        dJ.setZero(6, n);
        Eigen::Matrix6d dAdT, AdT;
        Eigen::Matrix4d invT, dInvT, tform;
        for (int i = n - 1; i >= 0; i--)
        {
            Eigen::Vector6d Ai(robot->A.at(i, 0), robot->A.at(i, 1), robot->A.at(i, 2),
                               robot->A.at(i, 3), robot->A.at(i, 4), robot->A.at(i, 5));

            Eigen::Map<const Eigen::Matrix4d> Mi(robot->M.data() + i * 16);
            derivative_tform_inv(T, dT, dInvT, invT);
            derivative_adjoint_T(invT, dInvT, dAdT, AdT);
            J.col(i) = AdT * Ai;
            dJ.col(i) = dAdT * Ai;
            tform = Mi * exp_twist(Ai * q[i]);
            dT = tform * (se_twist(Ai) * T * dq[i] + dT);
            T = tform * T;
        }
    }

    void gravity_and_inertia_compensation(const Robot &robot, const std::vector<double> &q, const std::vector<double> &qd, const std::vector<double> &qdd, double _T[16], double _Tcb[16], double _Tcs[16], float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double> &pose, const double _G[36]) // force:fxyzTxyz _Tcs:sensor frame based on gravity center frame
    {
        get_ext_force(force, mass, offset, cog, pose);
        double _Vb[6]{0}, _dVb[6]{0};
        get_task_space_motion(robot, q, qd, qdd, _T, _Vb, _dVb);
        Eigen::Vector6d Vc = Eigen::Vector6d::Zero();
        Eigen::Vector6d dVc = Eigen::Vector6d::Zero();
        Eigen::Map<const Eigen::Vector6d> Vb(_Vb);
        Eigen::Map<const Eigen::Vector6d> dVb(_dVb);
        Eigen::Map<const Eigen::Matrix4d> Tcb(_Tcb);
        Vc = adjoint_T(Tcb) * Vb;
        dVc = adjoint_T(Tcb) * dVb;
        //  bodyTwist2SpatialTwist(_Tcb, _Vb, _dVb, _Vc, _dVc);
        Eigen::Map<const Eigen::Matrix6d> G(_G);
        Eigen::Map<const Eigen::Matrix4d> Tcs(_Tcs);

        Eigen::Vector6d Fc = Eigen::Vector6d::Zero();
        Eigen::Matrix6d adV = Eigen::Matrix6d::Zero();
        adV.block(0, 0, 3, 3) = so_w(Vc.topRows(3));
        adV.block(3, 0, 3, 3) = so_w(Vc.bottomRows(3));
        adV.block(3, 3, 3, 3) = so_w(Vc.topRows(3));
        Fc = G * dVc - adV.transpose() * G * Vc;
        Eigen::Vector6d Fsensor = Eigen::Vector6d::Zero();
        Fsensor = adjoint_T(Tcs).transpose() * Fc;
        force[0] -= static_cast<float>(Fsensor(3));
        force[1] -= static_cast<float>(Fsensor(4));
        force[2] -= static_cast<float>(Fsensor(5));
        force[3] -= static_cast<float>(Fsensor(0));
        force[4] -= static_cast<float>(Fsensor(1));
        force[5] -= static_cast<float>(Fsensor(2));
    }

    void get_task_space_motion(const Robot &robot, const std::vector<double> &q, const std::vector<double> &qd, const std::vector<double> &qdd, double T[16], double V[6], double dV[6])
    {
        int n = q.size();
        double dT[16] = {0};
        Eigen::Map<Eigen::Vector6d> twist(V), dtwist(dV);
        if (n == 7)
        {
            Eigen::Matrix6x7d Jb, dJb;
            Eigen::Map<const Eigen::Vector7d> js(&qd[0]), ja(&qdd[0]);
            ::derivative_jacobian_matrix(&robot, q, qd, coder_array_wrapper1(dJb), coder_array_wrapper2(Jb), dT, T);
            twist = Jb * js;
            dtwist = dJb * js + Jb * ja;
        }
        else if (n == 6)
        {
            Eigen::Matrix6d Jb, dJb;
            Eigen::Map<const Eigen::Vector6d> js(&qd[0]), ja(&qdd[0]);
            ::derivative_jacobian_matrix(&robot, q, qd, coder_array_wrapper1(dJb), coder_array_wrapper2(Jb), dT, T);
            twist = Jb * js;
            dtwist = dJb * js + Jb * ja;
        }
    }

    Eigen::Vector6d gravity_and_inertia_compensation(const Robot &robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Tsensor, const std::vector<double> &q, const std::vector<double> &qd,
                                                     const std::vector<double> &qdd, const double *rawForce, double mass, const double offset[6], const double cog[3], const Eigen::Matrix3d &mI, double scale)
    {
        Eigen::Vector3d com(cog[0], cog[1], cog[2]);
        Eigen::Vector3d Pcom = Tsensor.block(0, 3, 3, 1) + Tsensor.block(0, 0, 3, 3) * com;
        Eigen::Matrix4d Tcb = Eigen::Matrix4d::Identity();
        Tcb.block(0, 3, 3, 1) = -Pcom;
        Eigen::Matrix6d adTcb = adjoint_T(Tcb);
        Eigen::Matrix6d g = Eigen::Matrix6d::Identity();
        g(3, 3) = g(4, 4) = g(5, 5) = mass;
        g.topLeftCorner(3, 3) = mI;
        Eigen::Vector6d Vc, dVc, Vb, dVb;
        Eigen::Matrix4d T;
        get_task_space_motion(robot, q, qd, qdd, T.data(), Vb.data(), dVb.data());
        Vc = adTcb * Vb;
        dVc = adTcb * dVb;
        Eigen::Vector6d Ftotal = g * dVc - adjoint_V(Vc).transpose() * g * Vc;
        Eigen::Vector6d Fg(0, 0, 0, 0, 0, 0);
        Fg.bottomRows(3) = mass * 9.8 * T.block(0, 0, 3, 3).transpose() * Eigen::Vector3d(0, 0, -1);
        Eigen::Vector6d rawWrench(rawForce[3] - offset[3], rawForce[4] - offset[4], rawForce[5] - offset[5],
                                  rawForce[0] - offset[0], rawForce[1] - offset[1], rawForce[2] - offset[2]);
        Eigen::Vector6d Fsensor = adjoint_T(inv_tform(Tsensor)).transpose() * rawWrench * scale;
        Eigen::Vector6d Fext = adTcb.transpose() * (Fg - Ftotal) - Fsensor;
        return adjoint_T(Tcp).transpose() * Fext;
    }

    void admittance_error_cal(const Robot *robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Td, const Eigen::Vector6d &Vd,
                              const std::vector<double> &q, const std::vector<double> &qd, Eigen::Vector3d &re, Eigen::Vector3d &pe, Eigen::Vector3d &red, Eigen::Vector3d &ped, bool flag)
    {
        int n = robot->dof;
        Eigen::Matrix4d T;
        Eigen::MatrixX<double> Jb(6, n);
        coder::array<double, 2> Jb_array;
        Jb_array.set(Jb.data(), 6, n);
        ::jacobian_matrix(robot, q, Jb_array, T.data());
        T = T * Tcp;
        Jb = adjoint_T(inv_tform(Tcp)) * Jb;
        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        Eigen::Vector3d p = T.block(0, 3, 3, 1);
        Eigen::Matrix3d Rd = Td.block(0, 0, 3, 3);
        Eigen::Vector3d pd = Td.block(0, 3, 3, 1);
        Eigen::Vector6d V = Jb * Eigen::Map<const Eigen::MatrixX<double>>(&qd[0], n, 1);
        if (flag)
            pe = R.transpose() * (pd - p);
        ped = -so_w(V.topRows(3)) * pe + R.transpose() * Vd.bottomRows(3) - V.bottomRows(3);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R.transpose() * Rd, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d tem = svd.matrixU() * svd.matrixV().transpose();
        re = logR(tem);
        double re_norm = re.norm();
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        if (re_norm > 0)
        {
            Eigen::Matrix3d S = so_w(re);
            double re_norm2 = re_norm * re_norm;
            double re_norm3 = re_norm2 * re_norm;
            A = Eigen::Matrix3d::Identity() - (1 - cos(re_norm)) / re_norm2 * S + (re_norm - sin(re_norm)) / re_norm3 * S * S;
        }
        red = A.colPivHouseholderQr().solve(Rd.transpose() * Vd.topRows(3) - Rd.transpose() * R * V.topRows(3));
    }

    void admittance_control(const Robot *robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Td, const Eigen::Vector6d &Vd,
                            const Eigen::Matrix3d &Mp, const Eigen::Matrix3d &Bp, const Eigen::Matrix3d &Kp,
                            const Eigen::Matrix3d &Mr, const Eigen::Matrix3d &Br, const Eigen::Matrix3d &Kr,
                            const std::vector<double> &q, const std::vector<double> &qd, const Eigen::Vector6d &F, double dt,
                            Eigen::Vector3d &re, Eigen::Vector3d &pe, Eigen::Vector3d &red, Eigen::Vector3d &ped, bool flag, double *Tcmd)
    {

        admittance_error_cal(robot, Tcp, Td, Vd, q, qd, re, pe, red, ped, flag);

        Eigen::Vector3d pedd = Mp.ldlt().solve(F.bottomRows(3) - Bp * ped - Kp * pe);
        Eigen::Vector3d redd = Mr.ldlt().solve(F.topRows(3) - Br * red - Kr * re);

        ped = ped + pedd * dt;
        pe = pe + ped * dt;
        red = red + redd * dt;
        re = re + red * dt;
        if (Tcmd)
        {
            Eigen::Map<Eigen::Matrix4d> T2(Tcmd);
            T2 = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d Tbs = Eigen::Matrix4d::Identity();
            Tbs.block(0, 0, 3, 3) = Td.block(0, 0, 3, 3).transpose();
            Eigen::Vector6d Vdb = adjoint_T(Tbs) * Vd * dt;
            Eigen::Matrix4d dTd;
            dTd = exp_twist(Vdb);
            Eigen::Matrix4d Td2 = Td * dTd;
            T2.block(0, 0, 3, 3) = Td2.block(0, 0, 3, 3) * exp_r(-re);
            T2.block(0, 3, 3, 1) = Td2.block(0, 3, 3, 1) - T2.block(0, 0, 3, 3) * pe;
            T2 = T2 * inv_tform(Tcp);
        }
    }

} // namespace robot_math
