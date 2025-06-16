#include <iostream>
#include <Eigen/Dense>

#include "Manip2Dof.hpp"

Manip2Dof::Manip2Dof()
{
    std::cout << "Creating 2-DoF Manipulator Object\n";
}

void Manip2Dof::Fk(const double t1, const double t2)
{
    std::cout << "Forward Kinematics\n";
}

void Manip2Dof::Ik(const double x, const double y)
{
    std::cout << "Inverse Kinematics for " << x << "," << y << "\n";
}