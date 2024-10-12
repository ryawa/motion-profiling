#include "Path.hpp"

// TODO: maybe don't use row vector?
// TODO: header guards?
QuinticHermite::QuinticHermite(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1) {
    points << p0, v0, a0, p1, v1, a1;
    pointCoeffs =
        Eigen::MatrixXd {
            {-6, -3, -0.5, 6, -3, 0.5}, {15, 8, 1.5, -15, 7, -1}, {-10, -6, -1.5, 10, -4, 0.5},
            {0, 0, 0.5, 0, 0, 0},       {0, 1, 0, 0, 0, 0},       {1, 0, 0, 0, 0, 0},
        } *
        points;
    dCoeffs =
        Eigen::MatrixXd {
            {-30, -15, -2.5, 30, -15, 2.5},
            {60, 32, 6, -60, 28, -4},
            {-30, -18, -4.5, 30, -12, 1.5},
            {0, 0, 1, 0, 0, 0},
            {0, 1, 0, 0, 0, 0},
        } *
        points;
    d2Coeffs =
        Eigen::MatrixXd {
            {-120, -60, -10, 120, -60, 10},
            {180, 96, 18, -180, 84, -12},
            {-60, -36, -9, 60, -24, 3},
            {0, 0, 1, 0, 0, 0},
        } *
        points;
}

QuinticHermite::QuinticHermite(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1)
    : QuinticHermite(p0, v0, {0, 0}, p1, v1, {0, 0}) {}

Vec2 QuinticHermite::point(double t) {
    Eigen::RowVectorXd T {{
        t * t * t * t * t,
        t * t * t * t,
        t * t * t,
        t * t,
        t,
        1,
    }};
    return T * pointCoeffs;
}

Vec2 QuinticHermite::d(double t) {
    Eigen::RowVectorXd T {{
        t * t * t * t,
        t * t * t,
        t * t,
        t,
        1,
    }};
    return T * dCoeffs;
}

Vec2 QuinticHermite::d2(double t) {
    Eigen::RowVectorXd T {{
        t * t * t,
        t * t,
        t,
        1,
    }};
    return T * d2Coeffs;
}
