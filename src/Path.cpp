#include <cmath>
#include "Path.hpp"
#include "Eigen/src/Core/Matrix.h"

CubicBezier::CubicBezier(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1) {
    points << p0, v0, p1, v1;
    pointCoeffs = points * Eigen::Matrix<float, 4, 4> {
                               {-1, 3, -3, 1},
                               {3, -6, 3, 0},
                               {-3, 3, 0, 0},
                               {1, 0, 0, 0},
                           };
    dCoeffs = points * Eigen::Matrix<float, 4, 3> {
                           {-3, 6, -3},
                           {9, -12, 3},
                           {-9, 6, 0},
                           {3, 0, 0},
                       };
    d2Coeffs = points * Eigen::Matrix<float, 4, 2> {
                            {-6, 6},
                            {18, -12},
                            {-18, 6},
                            {6, 0},
                        };
}

Vec2 CubicBezier::point(float t) {
    Eigen::Vector<float, 4> T;
    T << t * t * t, t * t, t, 1;
    return pointCoeffs * T;
}

Vec2 CubicBezier::d(float t) {
    Eigen::Vector<float, 3> T;
    T << t * t, t, 1;
    return dCoeffs * T;
}

Vec2 CubicBezier::d2(float t) {
    Eigen::Vector<float, 2> T;
    T << t, 1;
    return d2Coeffs * T;
}

QuinticHermite::QuinticHermite(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1) {
    points << p0, v0, a0, p1, v1, a1;

    pointCoeffs = points * Eigen::Matrix<float, 6, 6> {
                               {-6, 15, -10, 0, 0, 1}, {-3, 8, -6, 0, 1, 0}, {-0.5, 1.5, -1.5, 0.5, 0, 0},
                               {6, -15, 10, 0, 0, 0},  {-3, 7, -4, 0, 0, 0}, {0.5, -1, 0.5, 0, 0, 0},
                           };

    dCoeffs = points * Eigen::Matrix<float, 6, 5> {
                           {-30, 60, -30, 0, 0}, {-15, 32, -18, 0, 1}, {-2.5, 6, -4.5, 1, 0},
                           {30, -60, 30, 0, 0},  {-15, 28, -12, 0, 0}, {2.5, -4, 1.5, 0, 0},
                       };

    d2Coeffs = points * Eigen::Matrix<float, 6, 4> {
                            {-120, 180, -60, 0}, {-60, 96, -36, 1}, {-10, 18, -9, 1},
                            {120, -180, 60, 0},  {-60, 84, -24, 0}, {10, -12, 3, 0},
                        };
}

QuinticHermite::QuinticHermite(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1)
    : QuinticHermite(p0, v0, Vec2::Zero(), p1, v1, Vec2::Zero()) {}

Vec2 QuinticHermite::point(float t) {
    Eigen::Vector<float, 6> T;
    T << t * t * t * t * t, t * t * t * t, t * t * t, t * t, t, 1;
    return pointCoeffs * T;
}

Vec2 QuinticHermite::d(float t) {
    Eigen::Vector<float, 5> T;
    T << t * t * t * t, t * t * t, t * t, t, 1;
    return dCoeffs * T;
}

Vec2 QuinticHermite::d2(float t) {
    Eigen::Vector<float, 4> T;
    T << t * t * t, t * t, t, 1;
    return d2Coeffs * T;
}

float QuinticHermite::convexHull() {
    Vec2 q1 = points.col(0) + points.col(1) / 3;
    Vec2 q2 = points.col(3) - points.col(4) / 3;
    Vec2 a = q1 - points.col(0);
    Vec2 b = q2 - q1;
    Vec2 c = points.col(3) - q2;
    Vec2 d = points.col(0) - points.col(3);
    return a.norm() + b.norm() + c.norm() + d.norm();
}

QuinticHermiteSpline::QuinticHermiteSpline(std::vector<QuinticHermite> segments)
    : segments(segments) {};

Vec2 QuinticHermiteSpline::point(float t) {
    int num = std::ceil(t) - 1;
    if (num < 0) { num = 0; }
    float frac = t - num;
    QuinticHermite segment = segments[num];
    return segment.point(frac);
}

Vec2 QuinticHermiteSpline::d(float t) {
    int num = std::ceil(t) - 1;
    if (num < 0) { num = 0; }
    float frac = t - num;
    QuinticHermite segment = segments[num];
    return segment.d(frac);
}

Vec2 QuinticHermiteSpline::d2(float t) {
    int num = std::ceil(t) - 1;
    if (num < 0) { num = 0; }
    float frac = t - num;
    QuinticHermite segment = segments[num];
    return segment.d2(frac);
}
