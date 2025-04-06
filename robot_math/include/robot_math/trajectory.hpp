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
        void evaluate(double t, Eigen::Matrix4d &Td, Eigen::Vector6d &V, Eigen::Vector6d &dV)
        {
            std::vector<double> p(6), v(6), a(6);
            for (int k = 0; k < 6; k++)
            {
                p[k] = spl[k](t);
                v[k] = spl[k].deriv(1, t);
                a[k] = spl[k].deriv(2, t);
            }
            Td = pose_to_tform(p);
            V.tail(3) = Eigen::Vector3d(v[0], v[1], v[2]);
            dV.tail(3) = Eigen::Vector3d(a[0], a[1], a[2]);
            Eigen::Matrix3d R = Td.block(0, 0, 3, 3);
            Eigen::Vector3d r(p[3], p[4], p[5]);
            Eigen::Vector3d dr(v[3], v[4], v[5]);
            Eigen::Vector3d ddr(a[3], a[4], a[5]);
            Eigen::Matrix3d Ar = A_r(r);
            Eigen::Vector3d wb = Ar * dr;
            Eigen::Matrix3d dR = R * so_w(wb);
            V.head(3) = R * Ar * dr;
            dV.head(3) = dR * wb + R * dA_r(r, dr) * dr + R * Ar * ddr;
        }
        void set_traj(const std::vector<double> &traj)
        {
            num_of_point_ = traj.size() / 7;
            y_.resize(6);
            for (std::size_t i = 0; i < num_of_point_; i++)
            {
                time_.push_back(traj[i * 7]);
                for (int k = 0; k < 6; k++)
                {
                    y_[k].push_back(traj[i * 7 + k + 1]);
                }
            }
            for (int k = 0; k < 6; k++)
            {
                spl[k].set_boundary(tk::spline::first_deriv, 0.0,
                                    tk::spline::first_deriv, 0.0);
                spl[k].set_points(time_, y_[k]);
                spl[k].make_monotonic();
            }
        }

    protected:
        double T;
        tk::spline spl[6];
        std::vector<std::vector<double>> y_;
        std::vector<double> time_;
        std::size_t num_of_point_ = 0;
    };

}