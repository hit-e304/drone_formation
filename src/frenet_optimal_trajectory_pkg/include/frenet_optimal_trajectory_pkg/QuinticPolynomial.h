//
// Created by dqn on 19-4-20.
//

#ifndef CLASS_TEST_QUINITIC_POLYNOMIAL_H
#define CLASS_TEST_QUINITIC_POLYNOMIAL_H


#include <Eigen/Dense>
#include <Eigen/LU>

using namespace Eigen;

class QuinticPolynomial {
private:
    double a0_;
    double a1_;
    double a2_;
    double a3_;
    double a4_;
    double a5_;

public:
    double xs;
    double vxs;
    double axs;
    double xe;
    double vxe;
    double axe;
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T);
    virtual ~QuinticPolynomial();
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
};


#endif //CLASS_TEST_QUINITIC_POLYNOMIAL_H
