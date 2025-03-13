#include "ros2_utility/symbolic_diffentiator.hpp"

using namespace GiNaC;

SymbolicDifferentiator::SymbolicDifferentiator(const ex &expr, const symbol &x, const symbol &y, const symbol &z)
    : x_(x), y_(y), z_(z), expr_(expr)
{
    compute_derivatives();
}

ex SymbolicDifferentiator::derivative(const ex &expr, const symbol &var)
{
    return expr.diff(var);
}

void SymbolicDifferentiator::compute_derivatives()
{
    // 统一导数计算
    const symbol vars[] = {x_, y_, z_};

    // 计算梯度
    for (int i = 0; i < 3; ++i)
        gradient_[i] = derivative(expr_, vars[i]);

    // 计算Hessian
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            hessian_[i][j] = derivative(gradient_[i], vars[j]);
}

Eigen::Vector3d SymbolicDifferentiator::compute_gradient(const Eigen::Vector3d &point)
{
    Eigen::Vector3d grad;
    for (int i = 0; i < 3; ++i)
    {
        ex substituted = gradient_[i].subs(lst{x_ == point[0], y_ == point[1], z_ == point[2]}).evalf();
        grad[i] = ex_to<numeric>(substituted).to_double();
    }
    return grad;
}

Eigen::Matrix3d SymbolicDifferentiator::compute_hessian(const Eigen::Vector3d &point)
{
    Eigen::Matrix3d hess;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            ex substituted = hessian_[i][j].subs(lst{x_ == point[0], y_ == point[1], z_ == point[2]}).evalf();
            hess(i, j) = ex_to<numeric>(substituted).to_double();
        }
    }
    return hess;
}

double SymbolicDifferentiator::compute_value(const Eigen::Vector3d &point)
{
    ex substituted = expr_.subs(lst{x_ == point[0], y_ == point[1], z_ == point[2]}).evalf();
    return ex_to<numeric>(substituted).to_double();
}

void SymbolicDifferentiator::print_symbolic_results() const
{
    std::cout << "==== Symbolic Gradient ====\n[ ";
    for (int i = 0; i < 3; ++i)
    {
        std::cout << gradient_[i];
        if (i < 2)
            std::cout << ",\n  ";
    }
    std::cout << " ]\n\n";

    std::cout << "==== Symbolic Hessian ====\n";
    for (int i = 0; i < 3; ++i)
    {
        std::cout << "[ "; // 每行统一以[开头
        for (int j = 0; j < 3; ++j)
        {
            std::cout << std::left << hessian_[i][j];
            if (j < 2)
                std::cout << ", ";
        }
        std::cout << " ]\n"; // 每行统一以]结尾
    }
    std::cout << "\n";
}