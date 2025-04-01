#pragma once
#include "robot_math/robot_math.hpp"
#include "robot_math/ScaleFunction.hpp"
namespace robot_math
{

    class CartesianTrajectoryPlanner
    {
    public:
        CartesianTrajectoryPlanner()
        {
        }
        bool evaluate(double t, Eigen::Matrix4d &T, Eigen::Vector6d &V, Eigen::Vector6d &dV)
        {
            double s, ds, dds;
            if (s_.evaluate(t, s, ds, dds))
            {
                T.block<3, 3>(0, 0) = Rs_ * exp_r(re_ * s);
                T.block<3, 1>(0, 3) = ps_ + pse_ * s;
                T.block<1, 4>(3, 0) << 0, 0, 0, 1;
                V.head(3) = Rsre_ * ds;
                V.tail(3) = pse_ * ds;
                dV.head(3) = Rsre_ * dds;
                dV.tail(3) = pse_ * dds;
                return true;
            }
            return false;
        }

        int generate(const Eigen::Matrix4d &Ts, const Eigen::Matrix4d &Te, double time)
        {
            s_.generate(time);
            Rs_ = Ts.block<3, 3>(0, 0);
            Re_ = Te.block<3, 3>(0, 0);
            ps_ = Ts.block<3, 1>(0, 3);
            pe_ = Te.block<3, 1>(0, 3);
            re_ = logR(Rs_.transpose() * Re_);
            Rsre_ = Rs_ * re_;
            pse_ = pe_ - ps_;
            time_ = time;
        }

    protected:
        double time_;
        ScaleFunction s_;
        Eigen::Vector3d ps_, pe_, re_, pse_, Rsre_;
        Eigen::Matrix3d Rs_, Re_;
    };
}
