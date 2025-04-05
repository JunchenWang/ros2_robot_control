#pragma once
#include "robot_math/spline.h"
#include <vector>

namespace robot_math
{
    class CartesianTrajectory
    {
    public:
        CartesianTrajectory(const std::vector<double> &traj)
        {
            set_traj(traj);
        }
        void evaluate(double t, double p[], double v[], double a[])
        {
            for (int k = 0; k < 6; k++)
            {
                p[k] = spl[k](t);
                v[k] = spl[k].deriv(1, t);
                a[k] = spl[k].deriv(2, t);
            }
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
        std::size_t num_of_point_;
    };

}