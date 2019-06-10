#include <iostream>
#include "frenet_optimal_trajectory_pkg/CubicSplinePlanner.h"


using namespace CubicSplinePlanner;

std::vector<double> diff(std::vector<double> a) {
    std::vector<double> a_diff;
    for (int i = 0; i < a.size() - 1; i++) {
        a_diff.push_back(a[i + 1] - a[i]);
    }

    return a_diff;
}
int bisect(std::vector<double> a, double x)
{
    int lo = 0;
    int hi = a.size();
    int mid;
    while (lo < hi)
    {
        mid = (lo + hi) / 2;
        if (x < a[mid]) hi = mid;
        else lo = mid + 1;
    }
    return lo;
}
std::vector<double> cumsum(std::vector<double> list)
{
    std::vector<double> cum_list = list;
    double sum = 0;
    for (int i = 0; i < cum_list.size(); ++i) {
        sum = sum + list[i];
        cum_list[i] = sum;
    }
    return cum_list;
}

Spline::Spline(const std::vector<double> x, const std::vector<double> y)
{
    x_ = x;
    y_ = y;
    nx_ = x.size();
    h_ = diff(x_);
    double tb;
    for (int i = 0; i < y.size(); i++)
    {
        a_.push_back(y[i]);
    }
    A_ = __calc_A(h_);
    B_ = __calc_B(h_);
    c_ = A_.lu().solve(B_);

    for (int i = 0; i < nx_ - 1; ++i) {
        d_.push_back((c_(i + 1) - c_(i)) / (3.0 * h_[i]));
        tb = (a_[i + 1] - a_[i]) / h_[i] - h_[i] * (c_(i + 1) + 2.0 * c_(i)) / 3.0;
        b_.push_back(tb);
    }
    // ///////////////////////测试///////////////////////////
//    std::cout << A_ << std::endl;
//    std::cout << B_ << std::endl;
//    std::cout << c_ << std::endl;
}
double Spline::calc(double t) {
    // if (t < x_[0]) return NULL;
    // else if(t > x_.back()) return NULL;
    int i = __search_index(t);
    double dx = t - x_[i];
    double result = a_[i] + b_[i] * dx + c_(i) * dx * dx + d_[i] * dx * dx * dx;
    return result;
}
double Spline::calcd(double t) {
    // if (t < x_[0]) return NULL;
    // else if(t > x_.back()) return NULL;
    int i = __search_index(t);
    double dx = t - x_[i];
    double result = b_[i] + 2.0 * c_(i) * dx + 3.0 * d_[i] * dx * dx;
    return result;
}
double Spline::calcdd(double t)
{
    // if (t < x_[0]) return NULL;
    // else if(t > x_.back()) return NULL;
    int i = __search_index(t);
    double dx = t - x_[i];
    double result = 2.0 * c_(i) + 6.0 * d_[i] * dx;
    return result;
}

int Spline::__search_index(double num){//使用这个函数之前必须确保list是已排序的才行
    return bisect(x_, num) - 1;
}

MatrixXd Spline::__calc_A(std::vector<double> h)
{
    MatrixXd A = MatrixXd::Zero(nx_,nx_);
    A(0, 0) = 1.0;
    for (int i = 0; i < nx_ - 1; i++)
    {
        if (i != nx_ - 2)
        {
            A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
        }
        A(i + 1, i) = h[i];
        A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx_ - 1, nx_ - 2) = 0.0;
    A(nx_ - 1, nx_ - 1) = 1.0;
    return A;
}
VectorXd Spline::__calc_B(std::vector<double> h)
{
    VectorXd B = VectorXd::Zero(nx_);
    for (int i = 0; i < nx_ - 2; ++i) {
        B(i + 1) = 3.0 * (a_[i + 2] - a_[i + 1]) / h[i + 1] - 3.0 * (a_[i + 1] - a_[i]) / h[i];
    }
    return B;
}

Spline::~Spline()
{

}

Spline2D::Spline2D(const std::vector<double> x, const std::vector<double> y):sx_(__calc_s(x, y), x),sy_(__calc_s(x, y), y) {
    s = __calc_s(x, y);
//    sx_ = Spline(s_, x);
//    Spline sx_(s_, x);
//    sy_ = Spline(s_, y);
//    Spline sy_(s_, y);
}

std::vector<double> Spline2D::__calc_s(std::vector<double> x, std::vector<double> y)
{
    std::vector<double> dx = diff(x);
    std::vector<double> dy = diff(y);
    std::vector<double> ds;
    double ids;
    for (int i = 0; i < dx.size(); ++i) {
        ids = sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
        ds.push_back(ids);
    }
    std::vector<double> cum_ds = cumsum(ds);
    std::vector<double> s = {0};
    s.insert(s.end(), cum_ds.begin(), cum_ds.end());
    return s;
}

std::vector<double> Spline2D::calc_position(double s) {
    double x = sx_.calc(s);
    double y = sy_.calc(s);
    std::vector<double> xy;
    xy.push_back(x);
    xy.push_back(y);
    return xy;
}
double Spline2D::calc_curvature(double s) {
    double dx = sx_.calcd(s);
    double ddx = sx_.calcdd(s);
    double dy = sy_.calcd(s);
    double ddy = sy_.calcdd(s);
    double k = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
    return k;
}
double Spline2D::calc_yaw(double s) {
    double dx = sx_.calcd(s);
    double dy = sy_.calcd(s);
    double yaw = atan2(dy, dx);
    return yaw;
}

Spline2D::~Spline2D() {

}