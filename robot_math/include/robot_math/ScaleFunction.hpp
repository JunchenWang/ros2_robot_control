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
        ScaleFunction(): T_(.0), a_(.0), b_(.0)
        {
        }
        virtual ~ScaleFunction() {}
        virtual void evaluate(double t, double &s, double &ds, double &dds) = 0;
        // a: acceleration, b: initial velocity
        virtual void generate(double T, double a = 0, double b = 0) = 0;
    protected:
        double T_;
        double a_;
        double b_;
    };
}