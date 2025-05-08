#pragma once
#include <cmath>
namespace robot_math
{

    // ScaleFunction class is used to generate a smooth trajectory
    // with acceleration and deceleration phases.
    // The value is 0 ~ 1, and the time is 0 ~ T.
    class ScaleFunction
    {
    public:
        void evaluate(double t, double &s, double &ds, double &dds)
        {
            if(t < 0) 
            {
                t = 0;
            }
            else if(t > T_)
            {
                t = T_;
            }
            
            if (t < t1_)
            {
                s = flag_ * a_ * t * t / 2 + b_ * t;
                ds = flag_ * a_ * t + b_;
                dds = flag_ * a_;
            }
            else if (t < t2_)
            {
                s = offset_ + (t - t1_) * v_;
                ds = v_;
                dds = 0;
            }
            else
            {
                s = 1 - a_ * (T_ - t) * (T_ - t) / 2;
                ds = a_ * (T_ - t);
                dds = -a_;
            }
        }
        void generate(double T)
        {
            // average velocity
            double av = 2 * 1.0 / T;
            // acceleration is achievd in 0.2 T
            double a = av / T * 5;
            generate(T, a);
        }
        // T is total time, a is acceleration, b is initial velocity
        void generate(double T, double a, double b = 0)
        {
            if(T <= 0)
               return;

            double T2 = T * T;
            double delta = (2 * b * T - 4) * (2 * b * T - 4) + 4 * T2 * b * b;
            double amin = ((4 - 2 * b * T) + std::sqrt(delta)) / (2 * T2); // feasible minimum a
            if (amin > a)
                a = amin;

            double A = a, B = -(a * T + b), C = 1 + b * b / (2 * a);
            delta = B * B - 4 * A * C;
            double tb = (-B - std::sqrt(delta)) / (2 * A);
            v_ = a * tb;
            t1_ = (v_ - b) / a;
            if (t1_ < 0)
            {
                tb = (1 - b * b / (2 * a)) / (a * T - b);
                v_ = a * tb;
                t1_ = (v_ - b) / -a;
                flag_ = -1;
            }
            else
                flag_ = 1;

            offset_ = 0.5 * flag_ * a * t1_ * t1_ + b * t1_;
            t2_ = T - tb;
            T_ = T, a_ = a, b_ = b;
        }

    protected:
        double t1_ = 0;
        double t2_ = 0;
        double a_ = 0;
        double b_ = 0;
        double v_ = 0;
        double T_ = 0;
        double offset_ = 0;
        int flag_ = 1;
    };

}