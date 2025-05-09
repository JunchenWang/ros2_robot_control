#pragma once
#include "robot_math/robot_math.hpp"
#include "robot_math/ScaleFunction.hpp"
namespace robot_math
{

    class CartesianTrajectoryPlanner
    {
    public:
        CartesianTrajectoryPlanner(double time_threshod = 0.04) : time_threshod_(time_threshod), is_valid_(false)
        {
        }
        void set_time_threshold(double time_threshold)
        {
            time_threshod_ = time_threshold;
        }
        bool has_same_goal(const std::vector<double> &goal)
        {
            Eigen::Matrix4d G = pose_to_tform(goal);
            Eigen::Vector3d re = logR(Re_.transpose() * G.block<3, 3>(0, 0));
            Eigen::Vector3d pe = G.block<3, 1>(0, 3) - pe_;
            if (re.norm() < 1e-5 && pe.norm() < 1e-5)
                return true;
            return false;
        }
        void reset()
        {
            is_valid_ = false;
        }
        bool is_valid()
        {
            return is_valid_;
        }
        void evaluate(double t, Eigen::Matrix4d &T, Eigen::Vector6d &V, Eigen::Vector6d &dV)
        {
            double s, ds, dds;
            s_.evaluate(t, s, ds, dds);
            T.block<3, 3>(0, 0) = Rs_ * exp_r(re_ * s);
            T.block<3, 1>(0, 3) = ps_ + pse_ * s;
            T.block<1, 4>(3, 0) << 0, 0, 0, 1;
            V.head(3) = Rsre_ * ds;
            V.tail(3) = pse_ * ds;
            dV.head(3) = Rsre_ * dds;
            dV.tail(3) = pse_ * dds;
        }

        void generate_speed(const Eigen::Matrix4d &Ts, const Eigen::Matrix4d &Te, double v)
        {
            auto t1 = (Ts.block<3, 1>(0, 3) - Te.block<3, 1>(0, 3)).norm() / v;
            auto t2 = logR(Ts.block<3, 3>(0, 0).transpose() * Te.block<3, 3>(0, 0)).norm() / v;
            time_ = t1 > t2 ? t1 : t2;
            if (time_ < time_threshod_)
                time_ = time_threshod_;
            generate(Ts, Te, time_);
        }
        void generate(const Eigen::Matrix4d &Ts, const Eigen::Matrix4d &Te, double time)
        {
            Rs_ = Ts.block<3, 3>(0, 0);
            Re_ = Te.block<3, 3>(0, 0);
            ps_ = Ts.block<3, 1>(0, 3);
            pe_ = Te.block<3, 1>(0, 3);
            re_ = logR(Rs_.transpose() * Re_);
            Rsre_ = Rs_ * re_;
            pse_ = pe_ - ps_;
            time_ = time;
            s_.generate(time_);
            is_valid_ = true;
        }

    protected:
        double time_;
        double time_threshod_ = 0.04;
        ScaleFunction s_;
        Eigen::Vector3d ps_, pe_, re_, pse_, Rsre_;
        Eigen::Matrix3d Rs_, Re_;
        bool is_valid_;
    };
}
