//
// Created by dqn on 19-4-20.
//

#ifndef CLASS_TEST_QUARTICPOLYNOMIAL_H
#define CLASS_TEST_QUARTICPOLYNOMIAL_H


#include <Eigen/Dense>
#include <Eigen/LU>

using namespace Eigen;

class QuarticPolynomial {
private:
    double a0_;
    double a1_;
    double a2_;
    double a3_;
    double a4_;
public:
    double xs;
    double vxs;
    double axs;
    double vxe;
    double axe;
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double T);
    virtual ~QuarticPolynomial();
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
};


#endif //CLASS_TEST_QUARTICPOLYNOMIAL_H
