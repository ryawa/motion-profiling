#include <cmath>
#include "Trajectory.hpp"

Trajectory::Trajectory(QuinticHermite path, Constraints constraints, double step)
    : path(path),
      constraints(constraints),
      step(step) {};

void Trajectory::generate(double startVel, double endVel, double startAngularVel, double endAngularVel) {
    // TODO: iterative generation to avoid vector resizing
    double t = 0;
    Vec2 d = path.d(t);
    double theta = std::atan2(d[1], d[0]);
    double vel = startVel;
    double prevAngularVel = startAngularVel;

    while (t < 1) {
        t += step / std::hypot(d[0], d[1]);
        d = path.d(t);
        double nextTheta = std::atan2(d[1], d[0]);
        double curvature = (nextTheta - theta) / step;
        double angularVel = curvature * vel;
        theta = nextTheta;
        vels.push_back(vel);
        angularVels.push_back(angularVel);

        double angularAcc = (angularVel - prevAngularVel) * (vel / step);
        prevAngularVel = angularVel;
        double maxAcc = constraints.maxAcc - std::abs(angularAcc * constraints.trackWidth / 2);
        vel = std::min(constraints.maxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAcc * step));
    }

    t = 1;
    d = path.d(t);
    vel = endVel;
    prevAngularVel = endAngularVel;

    vels.push_back(vel);
    angularVels.push_back(0);

    int i = vels.size() - 1;
    while (t > 0) {
        t -= step / std::hypot(d[0], d[1]);
        d = path.d(t);
        double nextTheta = std::atan2(d[1], d[0]);
        double curvature = -(nextTheta - theta) / step;
        double angularVel = curvature * vel;
        theta = nextTheta;
        if (std::abs(vel) < std::abs(vels[i])) { vels[i] = vel; }
        if (std::abs(angularVel) < std::abs(angularVels[i])) { angularVels[i] = angularVel; }
        i--;

        double angularAcc = (angularVel - prevAngularVel) * (vel / step);
        prevAngularVel = angularVel;
        double maxAcc = constraints.maxDec - std::abs(angularAcc * constraints.trackWidth / 2);
        vel = std::min(constraints.maxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAcc * step));
    }
}
