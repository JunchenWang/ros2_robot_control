//
// File: sqrt.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Apr-2023 20:33:26
//

// Include Files
#include "sqrt.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Declarations
static double rt_hypotd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
    double a;
    double y;
    a = std::abs(u0);
    y = std::abs(u1);
    if (a < y) {
        a /= y;
        y *= std::sqrt(a * a + 1.0);
    }
    else if (a > y) {
        y /= a;
        y = a * std::sqrt(y * y + 1.0);
    }
    else {
        if (!rtIsNaN(y)) {
            y = a * 1.4142135623730951;
        }
    }

    return y;
}

//
// Arguments    : creal_T *x
// Return Type  : void
//
namespace coder
{
    namespace internal
    {
        namespace scalar
        {
            void b_sqrt(creal_T* x)
            {
                double absxi;
                double absxr;
                double xi;
                double xr;
                xr = x->re;
                xi = x->im;
                if (xi == 0.0) {
                    if (xr < 0.0) {
                        absxi = 0.0;
                        xr = std::sqrt(-xr);
                    }
                    else {
                        absxi = std::sqrt(xr);
                        xr = 0.0;
                    }
                }
                else if (xr == 0.0) {
                    if (xi < 0.0) {
                        absxi = std::sqrt(-xi / 2.0);
                        xr = -absxi;
                    }
                    else {
                        absxi = std::sqrt(xi / 2.0);
                        xr = absxi;
                    }
                }
                else if (rtIsNaN(xr)) {
                    absxi = xr;
                }
                else if (rtIsNaN(xi)) {
                    absxi = xi;
                    xr = xi;
                }
                else if (rtIsInf(xi)) {
                    absxi = std::abs(xi);
                    xr = xi;
                }
                else if (rtIsInf(xr)) {
                    if (xr < 0.0) {
                        absxi = 0.0;
                        xr = xi * -xr;
                    }
                    else {
                        absxi = xr;
                        xr = 0.0;
                    }
                }
                else {
                    absxr = std::abs(xr);
                    absxi = std::abs(xi);
                    if ((absxr > 4.4942328371557893E+307) || (absxi >
                        4.4942328371557893E+307)) {
                        absxr *= 0.5;
                        absxi = rt_hypotd_snf(absxr, absxi * 0.5);
                        if (absxi > absxr) {
                            absxi = std::sqrt(absxi) * std::sqrt(absxr / absxi + 1.0);
                        }
                        else {
                            absxi = std::sqrt(absxi) * 1.4142135623730951;
                        }
                    }
                    else {
                        absxi = std::sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
                    }

                    if (xr > 0.0) {
                        xr = 0.5 * (xi / absxi);
                    }
                    else {
                        if (xi < 0.0) {
                            xr = -absxi;
                        }
                        else {
                            xr = absxi;
                        }

                        absxi = 0.5 * (xi / xr);
                    }
                }

                x->re = absxi;
                x->im = xr;
            }
        }
    }
}

//
// File trailer for sqrt.cpp
//
// [EOF]
//
