#include <array>

class Manip2Dof
{
public:
    Manip2Dof();
    ~Manip2Dof(){}

    void Fk(const double t1, const double t2);
    void Ik(const double x, const double y);
    std::array<double, 2> GetJointConfiguration();

private:
    // Link lengths 1 and 2
    double l1 {3};
    double l2 {3};

    // Joint angles 1 and 2
    std::array<double, 2> mJointConfig {0,0};

    // EF position
    double x {0};
    double y {0};

};