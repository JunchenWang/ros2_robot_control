#pragma once
#include "robot_math/robot_math.hpp"
#include "robot_math/ScaleFunction.hpp"
namespace robot_math
{

    class JointTrajectoryPlanner
    {
    public:
        JointTrajectoryPlanner()
        {
        }
        bool has_same_goal(const std::vector<double> &goal)
        {
            if (q1_.size() != goal.size())
                return false;
            for (std::size_t i = 0; i < q1_.size(); i++)
            {
                if (fabs(q1_[i] - goal[i]) > 1e-6)
                    return false;
            }
            return true;
        }
        bool evaluate(double t, std::vector<double> &q, std::vector<double> &dq, std::vector<double> &ddq)
        {
            q.resize(n_);
            dq.resize(n_);
            ddq.resize(n_);
            double s, ds, dds;
            if (ss_.empty())
            {
                if (s_.evaluate(t, s, ds, dds))
                {
                    for (std::size_t i = 0; i < n_; i++)
                    {
                        q[i] = q0_[i] + (q1_[i] - q0_[i]) * s;
                        dq[i] = (q1_[i] - q0_[i]) * ds;
                        ddq[i] = (q1_[i] - q0_[i]) * dds;
                    }
                    return true;
                }
                return false;
            }
            else
            {
                for (std::size_t i = 0; i < n_; i++)
                {
                    if (ss_[i].evaluate(t, s, ds, dds))
                    {
                        q[i] = q0_[i] + (q1_[i] - q0_[i]) * s;
                        dq[i] = (q1_[i] - q0_[i]) * ds;
                        ddq[i] = (q1_[i] - q0_[i]) * dds;
                    }
                    else
                        return false;
                }
                return true;
            }
        }

        void generate(const std::vector<double> &q0, const std::vector<double> &dq0, const std::vector<double> &q1, double time)
        {
            q0_ = q0;
            q1_ = q1;
            n_ = q0.size();
            time_ = time;
            ss_.resize(n_);
            for (std::size_t i = 0; i < n_; i++)
            {
                double b = 0;
                if(fabs(q1[i] - q0[i]) > 1e-6)
                {
                    b = dq0[i] / (q1[i] - q0[i]);
                }
                double a = 2.0 / time_ / time_ * 5;
                ss_[i].generate(time_, a, b);
            }
        }
        void generate(const std::vector<double> &q0, const std::vector<double> &q1, double time)
        {
            q0_ = q0;
            q1_ = q1;
            n_ = q0.size();
            double a = 2.0 / time_ / time_ * 5;
            s_.generate(time, a, 0);
            time_ = time;
            ss_.clear();
        }

        void generate_speed(const std::vector<double> &q0, const std::vector<double> &q1, double v)
        {
            time_ = 0;
            for(std::size_t i = 0; i < q0.size(); i++)
            {
                double t = fabs(q1[i] - q0[i]) / v;
                if(t > time_)
                    time_ = t;
            }
            generate(q0, q1, time_);
        }
        void generate_speed(const std::vector<double> &q0, const std::vector<double> &dq0, const std::vector<double> &q1, double v)
        {
            time_ = 0;
            for(std::size_t i = 0; i < q0.size(); i++)
            {
                double t = fabs(q1[i] - q0[i]) / v;
                if(t > time_)
                    time_ = t;
            }
            generate(q0, dq0, q1, time_);
        }

    protected:
        double time_;
        std::size_t n_;
        ScaleFunction s_;
        std::vector<ScaleFunction> ss_;
        std::vector<double> q0_, q1_;
    };
}
