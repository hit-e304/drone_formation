//
// Created by dqn on 19-4-20.
//

#include "frenet_optimal_trajectory_pkg/QuinticPolynomial.h"

QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T) {
    Matrix3d A;
    Vector3d b;
    Vector3d x;
    xs = xs;
    vxs = vxs;
    axs = axs;
    xe = xe;
    vxe = vxe;
    axe = axe;
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0; //为啥要除以2.0呢
    A << pow(T, 3), pow(T, 4), pow(T, 5),
         3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
         6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
    b << xe - a0_ - a1_ * T - a2_ * pow(T, 2),
         vxe - a1_ - 2 * a2_ * T ,
         axe - 2 * a2_;
    x = A.lu().solve(b);
    a3_ = x(0);
    a4_ = x(1);
    a5_ = x(2);
}
double QuinticPolynomial::calc_point(double t) {
    double xt = a0_ + a1_ * t + a2_ * pow(t, 2) + a3_ * pow(t, 3) + a4_ * pow(t, 4) + a5_ * pow(t, 5);
    return xt;
}
double QuinticPolynomial::calc_first_derivative(double t) {
    double xt = a1_ + 2 * a2_ * t + 3 * a3_ * pow(t, 2) + 4 * a4_ * pow(t, 3) + 5 * a5_ * pow(t, 4);
    return xt;
}
double QuinticPolynomial::calc_second_derivative(double t) {
    double xt = 2 * a2_ + 6 * a3_ * t + 12 * a4_ * pow(t, 2) + 20 * a5_ * pow(t, 3);
    return xt;
}
double QuinticPolynomial::calc_third_derivative(double t) {
    double xt = 6 * a3_ + 24 * a4_ * t + 60 * a5_ * pow(t, 2);
    return xt;
}

QuinticPolynomial::~QuinticPolynomial() {

}
