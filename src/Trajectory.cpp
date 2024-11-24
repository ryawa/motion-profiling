#include <cmath>
#include <numbers>
#include "Trajectory.hpp"

Trajectory::Trajectory(QuinticHermite path, Constraints constraints, double step)
    : path(path),
      constraints(constraints),
      step(step) {};

// TODO: Iterative generation
void Trajectory::generate(double startVel, double endVel, double startAngularVel, double endAngularVel) {
    double t = 0;
    Vec2 d = path.d(t);
    double theta = std::atan2(d[1], d[0]);
    vels.push_back(startVel);
    angularVels.push_back(startAngularVel);
    double prevVel = startVel;
    double prevAngularVel = startAngularVel;

    while (t < 1) {
        t += step / std::hypot(d[0], d[1]);
        d = path.d(t);
        double nextTheta = std::atan2(d[1], d[0]);
        double curvature = constrainAngle90(nextTheta - theta) / step;

        double vel = std::min(constraints.constrainedSpeed(curvature),
                              std::sqrt(prevVel * prevVel + 2 * constraints.constrainedAcc(curvature) * step));
        double angularVel = curvature * vel;
        double acc = (vel - prevVel) * (prevVel / step);
        double angularAcc = (angularVel - prevAngularVel) * (prevVel / step);

        prevAngularVel = angularVel;
        prevVel = vel;
        theta = nextTheta;

        vels.push_back(vel);
        angularVels.push_back(angularVel);
        accels.push_back(acc);
        angularAccels.push_back(angularAcc);
    }

    t = 1;
    d = path.d(t);
    theta = std::atan2(d[1], d[0]);
    vels.push_back(endVel);
    angularVels.push_back(endAngularVel);
    accels.resize(vels.size());
    angularAccels.resize(vels.size());
    prevVel = endVel;
    prevAngularVel = endAngularVel;

    int i = vels.size() - 1;
    while (t > 0) {
        t -= step / std::hypot(d[0], d[1]);
        d = path.d(t);
        double nextTheta = std::atan2(d[1], d[0]);
        double curvature = constrainAngle90(nextTheta - theta) / step;
        double vel = std::min(constraints.constrainedSpeed(curvature),
                              std::sqrt(prevVel * prevVel + 2 * constraints.constrainedDec(curvature) * step));
        double angularVel = -curvature * vel;
        double acc = -(vel - prevVel) * (prevVel / step);
        double angularAcc = -(angularVel - prevAngularVel) * (prevVel / step);

        prevVel = vel;
        prevAngularVel = angularVel;
        theta = nextTheta;

        if (std::abs(vel) < std::abs(vels[i])) {
            vels[i] = vel;
            angularVels[i] = angularVel;
            accels[i] = acc;
            angularAccels[i] = angularAcc;
        }
        i--;
    }
}

double Trajectory::constrainAngle180(double angle) { return std::remainder(angle, 2 * std::numbers::pi); }

double Trajectory::constrainAngle90(double angle) {
    double constrained180 = constrainAngle180(angle);
    if (std::abs(constrained180) > std::numbers::pi / 2) {
        return constrainAngle180(constrained180 + std::numbers::pi);
    }
    return constrained180;
}
