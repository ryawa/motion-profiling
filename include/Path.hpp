#pragma once

#include "Eigen/Dense"

using Vec2 = Eigen::Vector2f;

class QuinticHermite {
    public:
        QuinticHermite(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1);
        QuinticHermite(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1);
        Vec2 point(float t);
        Vec2 d(float t);
        Vec2 d2(float t);
        float convexHull();
    private:
        Eigen::Matrix<float, 2, 6> points;
        Eigen::Matrix<float, 2, 6> pointCoeffs;
        Eigen::Matrix<float, 2, 5> dCoeffs;
        Eigen::Matrix<float, 2, 4> d2Coeffs;
};
