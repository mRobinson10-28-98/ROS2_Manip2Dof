#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "Manip2Dof.hpp"

Manip2Dof::Manip2Dof()
{
    std::cout << "Creating 2-DoF Manipulator Object\n";
}

Manip2Dof::Manip2Dof(double l1, double l2)
{
    std::cout << "Creating 2-DoF Manipulator Object\n";

    mLinkLengths[0] = l1;
    mLinkLengths[1] = l2;
}

void Manip2Dof::Fk(const double t1, const double t2)
{
    std::cout << "Forward Kinematics for: " << t1 << ", " << t2 << std::endl;
    
    double l1 = mLinkLengths[0];
    double l2 = mLinkLengths[1];

    x = l1 * cos(t1) + l2 * cos(t2);
    y = l1 * sin(t1) + l2 * sin(t2);
}

void Manip2Dof::Ik(const double x, const double y)
{
    std::cout << "Inverse Kinematics for " << x << ", " << y << std::endl;

    double l1 = mLinkLengths[0];
    double l2 = mLinkLengths[1];

    // Geometric solution
    mJointConfig[1] = acos((pow(x, 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2));
    mJointConfig[0] = atan(y/x) - atan((l2 * sin(mJointConfig[1])) / (l1 + l2 * cos(mJointConfig[1])));

    std::cout << "Theta 1, 2: " << mJointConfig[0] << ", " << mJointConfig[1] << std::endl;
}

std::array<double, 2> Manip2Dof::GetJointConfiguration()
{
    return mJointConfig;
}

std::array<double, 2> Manip2Dof::GetLinkLengths()
{
    return mLinkLengths;
}
