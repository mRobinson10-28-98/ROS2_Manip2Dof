class Manip2Dof
{
public:
    Manip2Dof();
    ~Manip2Dof(){}

    void Fk(const double t1, const double t2);
    void Ik(const double x, const double y);


private:
    // Link lengths 1 and 2
    double l1;
    double l2;

    // Joint angles 1 and 2
    double theta1;
    double theta2;

    // EF position
    double x;
    double y;

};