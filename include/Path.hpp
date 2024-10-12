#pragma once

#include "Eigen/Dense"

using Vec2 = Eigen::RowVector2d;

class QuinticHermite {
    public:
        QuinticHermite(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1);
        QuinticHermite(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1);
        Vec2 point(double t);
        Vec2 d(double t);
        Vec2 d2(double t);
    private:
        Eigen::Matrix<double, 6, 2> points;
        Eigen::Matrix<double, 6, 2> pointCoeffs;
        Eigen::Matrix<double, 5, 2> dCoeffs;
        Eigen::Matrix<double, 4, 2> d2Coeffs;
};
