#ifndef CUBICSPLINEPLANNER_H_
#define CUBICSPLINEPLANNER_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>

using namespace Eigen;

namespace CubicSplinePlanner
{
    class Spline
    {
    public:
        Spline(const std::vector<double> x, const std::vector<double> y);
        virtual ~Spline();
        double calc(double t);
        double calcd(double t);
        double calcdd(double t);

    private:
        std::vector<double> x_;
        std::vector<double> y_;
        int nx_;
        std::vector<double> h_;
        std::vector<double> b_;
        VectorXd c_;
        std::vector<double> d_;
        std::vector<double> w_;
        std::vector<double> a_;
        MatrixXd A_;
        MatrixXd B_;

        MatrixXd __calc_A(std::vector<double> h);
        VectorXd __calc_B(std::vector<double> h);
        int __search_index(double num);
    };

    class Spline2D
    {
    public:
        Spline2D(const std::vector<double> x, const std::vector<double> y);
        virtual ~Spline2D();
        std::vector<double> calc_position(double s);
        double calc_curvature(double s);
        double calc_yaw(double s);
        std::vector<double> s;

    private:
        Spline sx_;
        Spline sy_;
//        std::vector<double> ds_;

        std::vector<double> __calc_s(std::vector<double> x, std::vector<double> y);
    };

}

#endif // CUBICSPLINEPLANNER_H_