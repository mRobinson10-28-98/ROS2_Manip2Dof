#include <array>

class Manip2Dof
{
public:
    Manip2Dof();
    Manip2Dof(double l1, double l2);
    ~Manip2Dof(){}

    void Fk(const double t1, const double t2);
    void Ik(const double x, const double y);
    std::array<double, 2> GetJointConfiguration();
    std::array<double, 2> GetLinkLengths();
    std::array<double, 2> GetEndPos();

private:
    // Link lengths 1 and 2
    std::array<double, 2> mLinkLengths {3, 3};

    // Joint angles 1 and 2
    std::array<double, 2> mJointConfig {0,0};

    // EF position
    std::array<double, 2> mEndPos {0,0};
};