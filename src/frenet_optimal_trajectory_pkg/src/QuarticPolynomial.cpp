//
// Created by dqn on 19-4-20.
//

#include "frenet_optimal_trajectory_pkg/QuarticPolynomial.h"
QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double T) {
    Matrix2d A;
    Vector2d b;
    Vector2d x;
    xs = xs;
    vxs = vxs;
    axs = axs;
    vxe = vxe;
    axe = axe;
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0; //为啥要除以2.0呢
    A << 3 * pow(T, 2), 4 * pow(T, 3),
         6 * T, 12 * pow(T, 2);
    b << vxe - a1_ - 2 * a2_ * T ,
         axe - 2 * a2_;
    x = A.lu().solve(b);
    a3_ = x(0);
    a4_ = x(1);
}

double QuarticPolynomial::calc_point(double t) {
    double xt = a0_ + a1_ * t + a2_ * pow(t, 2) + a3_ * pow(t, 3) + a4_ * pow(t, 4);
    return xt;
}

double QuarticPolynomial::calc_first_derivative(double t) {
    double xt = a1_ + 2 * a2_ * t + 3 * a3_ * pow(t, 2) + 4 * a4_ * pow(t, 3);
    return xt;
}

double QuarticPolynomial::calc_second_derivative(double t) {
    double xt = 2 * a2_ + 6 * a3_ * t + 12 * a4_ * pow(t, 2);
    return xt;
}

double QuarticPolynomial::calc_third_derivative(double t) {
    double xt = 6 * a3_ + 24 * a4_ * t;
    return xt;
}

QuarticPolynomial::~QuarticPolynomial() {

}