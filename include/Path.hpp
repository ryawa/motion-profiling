#pragma once

#include "Eigen/Dense"
#include <vector>

using Vec2 = Eigen::Vector2f;

class Path {
    public:
        Path() {};
        virtual ~Path() {};
        virtual Vec2 point(float t) = 0;
        virtual Vec2 d(float t) = 0;
        virtual Vec2 d2(float) = 0;
        virtual float max_t() = 0;
};

class CubicBezier : public Path {
    public:
        CubicBezier(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1);
        Vec2 point(float t);
        Vec2 d(float t);
        Vec2 d2(float t);

        float max_t() { return 1.0; }
    private:
        Eigen::Matrix<float, 2, 4> points;
        Eigen::Matrix<float, 2, 4> pointCoeffs;
        Eigen::Matrix<float, 2, 3> dCoeffs;
        Eigen::Matrix<float, 2, 2> d2Coeffs;
};

class QuinticHermite : public Path {
    public:
        QuinticHermite(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1);
        QuinticHermite(Vec2 p0, Vec2 v0, Vec2 p1, Vec2 v1);
        Vec2 point(float t);
        Vec2 d(float t);
        Vec2 d2(float t);

        float max_t() { return 1.0; }

        float convexHull();
    private:
        Eigen::Matrix<float, 2, 6> points;
        Eigen::Matrix<float, 2, 6> pointCoeffs;
        Eigen::Matrix<float, 2, 5> dCoeffs;
        Eigen::Matrix<float, 2, 4> d2Coeffs;
};

class QuinticHermiteSpline : public Path {
    public:
        QuinticHermiteSpline(std::vector<QuinticHermite> segments);
        Vec2 point(float t);
        Vec2 d(float t);
        Vec2 d2(float t);

        float max_t() { return segments.size(); }
    private:
        std::vector<QuinticHermite> segments;
};
