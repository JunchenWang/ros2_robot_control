#pragma once
#include "ScaleFunction.hpp"
namespace robot_math
{

    // ScaleFunction class is used to generate a smooth trajectory
    // with acceleration and deceleration phases.
    // The value is 0 ~ 1, and the time is 0 ~ T.
    class LinearFunction : public ScaleFunction
    {
    public:
        LinearFunction()
        {
        }
        void evaluate(double t, double &s, double &ds, double &dds) override
        {
            if (t < 0)
            {
                s = 0;
                ds = 0;
                dds = 0;
            }
            else if (t <= T_)
            {
                s = t / T_;
                ds = 1.0 / T_;
                dds = 0;
            }
            else
            {
                s = 1;
                ds = 0;
                dds = 0;
            }
        }
        void generate(double T, double a = 0, double b = 0) override
        {
            if(T < 0)
                return;
            T_ = T;
        }
        
    };
}