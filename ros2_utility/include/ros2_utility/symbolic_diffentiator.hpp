#ifndef SYMBOLIC_DIFFRENTIATOR
#define SYMBOLIC_DIFFRENTIATOR

#include <eigen3/Eigen/Dense>
#include <ginac/ginac.h>
#include <vector>

class SymbolicDifferentiator
{
public:
    SymbolicDifferentiator(const GiNaC::ex &expr, const GiNaC::symbol &x, const GiNaC::symbol &y, const GiNaC::symbol &z);

    Eigen::Vector3d compute_gradient(const Eigen::Vector3d &point);
    Eigen::Matrix3d compute_hessian(const Eigen::Vector3d &point);
    double compute_value(const Eigen::Vector3d& point);
    void print_symbolic_results() const;

private:
    void compute_derivatives();
    GiNaC::ex derivative(const GiNaC::ex &expr, const GiNaC::symbol &var);

    GiNaC::symbol x_, y_, z_;
    GiNaC::ex expr_;
    GiNaC::ex gradient_[3];
    GiNaC::ex hessian_[3][3];
};
#endif // SYMBOLIC_DIFFRENTIATOR