#include <cmath>
#include <numbers>
#include <utility>
#include "Trajectory.hpp"
#include <iostream>

Trajectory::Trajectory(Path* path, Constraints constraints, float step)
    : path(path),
      constraints(constraints),
      step(step) {};

// TODO: Iterative generation
void Trajectory::generate(float startVel, float endVel, float startAngularVel, float endAngularVel) {
    // float arcLengthBound = path.convexHull();
    // int n = (arcLengthBound + step - 1) / step;
    // vels.reserve(n);
    // curvatures.reserve(n);
    // desiredPoses.reserve(n);

    vels.push_back(startVel);
    if (startVel == 0) {
        curvatures.push_back(0);
    } else {
        curvatures.push_back(startAngularVel / startVel);
    }
    float vel = startVel;

    float t = 0;
    Vec2 d = path->d(t);
    float prevTheta = std::atan2(d[1], d[0]);
    t += step / std::hypot(d[0], d[1]);

    float theta;
    float curvature;

    while (t < path->max_t()) {
        d = path->d(t);
        theta = std::atan2(d[1], d[0]);
        curvature = constrainAngle90(theta - prevTheta) / step;

        vel = std::min(constraints.constrainedSpeed(curvature),
                       std::sqrt(vel * vel + 2 * constraints.constrainedAcc(curvature) * step));

        vels.push_back(vel);
        curvatures.push_back(curvature);

        Vec2 point = path->point(t);
        desiredPoses.push_back({point[0], point[1], theta});
        prevTheta = theta;
        t += step / std::hypot(d[0], d[1]);
    }

    d = path->d(1);
    Vec2 point = path->point(1);
    theta = std::atan2(d[1], d[0]);
    Pose end = {point[0], point[1], theta};
    desiredPoses.push_back(end);
    desiredPoses.push_back(end);

    vels.push_back(endVel);
    if (endVel == 0) {
        curvatures.push_back(0);
    } else {
        curvatures.push_back(endAngularVel / endVel);
    }
    vel = endVel;

    int i = vels.size() - 2;
    while (i >= 0) {
        curvature = curvatures[i];
        vel = std::min(constraints.constrainedSpeed(curvature),
                       std::sqrt(vel * vel + 2 * constraints.constrainedDec(curvature) * step));

        if (std::abs(vel) < std::abs(vels[i])) { vels[i] = vel; }
        i--;
    }
}

float Trajectory::constrainAngle180(float angle) { return std::remainder(angle, 2 * std::numbers::pi); }

float Trajectory::constrainAngle90(float angle) {
    float constrained180 = constrainAngle180(angle);
    if (std::abs(constrained180) > std::numbers::pi / 2) {
        return constrainAngle180(constrained180 + std::numbers::pi);
    }
    return constrained180;
}

std::pair<float, float> Trajectory::getWheelVelocities(int i, float wheelDiameter) {
    float vel = vels[i];
    float angularVel = vels[i] * curvatures[i];
    float leftVel = vel - angularVel * constraints.trackWidth / 2;
    float rightVel = vel + angularVel * constraints.trackWidth / 2;

    float leftVelRpm = leftVel / (std::numbers::pi * wheelDiameter) * 60;
    float rightVelRpm = rightVel / (std::numbers::pi * wheelDiameter) * 60;
    return std::make_pair(leftVelRpm, rightVelRpm);
}
