#pragma once
#include "robot_math/spline.h"
#include "robot_math/robot_math.hpp"
#include <vector>

namespace robot_math
{
    class CartesianTrajectory
    {
    public:
        CartesianTrajectory()
        {
        }
        CartesianTrajectory(const std::vector<double> &traj)
        {
            set_traj(traj);
        }
        bool is_empty()
        {
            return num_of_point_ == 0;
        }
        void clear()
        {
            y_.clear();
            time_.clear();
            num_of_point_ = 0;
        }
        double total_time()
        {
            return T_;
        }
        void evaluate(double t, Eigen::Matrix4d &Td, Eigen::Vector6d &V, Eigen::Vector6d &dV)
        {
           
            std::vector<double> p(3), v(3), a(3);
            for (int k = 0; k < 3; k++)
            {
                p[k] = spl_[k](t);
                v[k] = spl_[k].deriv(1, t);
                a[k] = spl_[k].deriv(2, t);
            }
            Td.block(0, 3, 3, 1) = Eigen::Vector3d(p[0], p[1], p[2]);
            V.tail(3) = Eigen::Vector3d(v[0], v[1], v[2]);
            dV.tail(3) = Eigen::Vector3d(a[0], a[1], a[2]);
            
            std::vector<double>::const_iterator it;
            it=std::upper_bound(time_.begin(),time_.end(),t);       // *it > x
            size_t idx = std::max( int(it-time_.begin())-1, 0);   // m_x[idx] <= x
            if(idx == num_of_point_ - 1)
            {
                Td.block(0, 0, 3, 3) = orientation_waypoint_[idx].toRotationMatrix();
                V.head(3) = Eigen::Vector3d(0, 0, 0);
                dV.head(3) = Eigen::Vector3d(0, 0, 0);
            }
            else
            {
                Eigen::Quaterniond q1 = orientation_waypoint_[idx];
                Eigen::Quaterniond q2 = orientation_waypoint_[idx + 1];
                if(q1.dot(q2) < 0)
                    q2.coeffs() = -q2.coeffs();
                double t1 = time_[idx];
                double t2 = time_[idx + 1];
                double dt = t2 - t1;
                double alpha = (t - t1) / dt;
                Eigen::Quaterniond q = q1.slerp(alpha, q2);
                Td.block(0, 0, 3, 3) = q.toRotationMatrix();
                V.head(3) = Eigen::Vector3d(0, 0, 0);
                dV.head(3) = Eigen::Vector3d(0, 0, 0);
            }
            // Eigen::Matrix3d R = Td.block(0, 0, 3, 3);
            // Eigen::Vector3d r(p[3], p[4], p[5]);
            // Eigen::Vector3d dr(v[3], v[4], v[5]);
            // Eigen::Vector3d ddr(a[3], a[4], a[5]);
            // Eigen::Matrix3d Ar = A_r(r);
            // Eigen::Vector3d wb = Ar * dr;
            // Eigen::Matrix3d dR = R * so_w(wb);
            // V.head(3) = R * Ar * dr;
            // dV.head(3) = dR * wb + R * dA_r(r, dr) * dr + R * Ar * ddr;
        }
        void set_traj(const std::vector<double> &traj)
        {
            num_of_point_ = traj.size() / 7;
            y_.clear();
            y_.resize(6);
            time_.clear();
            pos_waypoint_.clear();
            orientation_waypoint_.clear();
            for (std::size_t i = 0; i < num_of_point_; i++)
            {
                time_.push_back(traj[i * 7]);
                Eigen::Vector3d pos(traj[i * 7 + 1], traj[i * 7 + 2], traj[i * 7 + 3]);
                Eigen::Quaterniond orientation = rv_to_quaternion(Eigen::Vector3d(traj[i * 7 + 4], traj[i * 7 + 5], traj[i * 7 + 6]));
                pos_waypoint_.emplace_back(pos);
                orientation_waypoint_.emplace_back(orientation);
                for (int k = 0; k < 3; k++)
                {
                    y_[k].push_back(traj[i * 7 + k + 1]);
                }
            }
            for (int k = 0; k < 3; k++)
            {
                spl_[k].set_boundary(tk::spline::first_deriv, 0.0,
                                    tk::spline::first_deriv, 0.0);
                spl_[k].set_points(time_, y_[k]);
                spl_[k].make_monotonic();
            }
            T_ = time_.back() - time_.front();
        }

    protected:
        double T_;
        tk::spline spl_[3];
        std::vector<std::vector<double>> y_;
        std::vector<double> time_;
        std::size_t num_of_point_ = 0;
        std::vector<Eigen::Vector3d> pos_waypoint_;
        std::vector<Eigen::Quaterniond> orientation_waypoint_;

    };

}