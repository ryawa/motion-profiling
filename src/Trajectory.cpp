#include <cmath>
#include <numbers>
#include "Trajectory.hpp"

Trajectory::Trajectory(QuinticHermite path, Constraints constraints, double step)
    : path(path),
      constraints(constraints),
      step(step) {};

// TODO: Iterative generation
// TODO: Start/end accels
// TODO: Use curvature to get angular values
void Trajectory::generate(double startVel, double endVel, double startAngularVel, double endAngularVel) {
    // double t = 0;
    // Vec2 d = path.d(t);
    // double theta = std::atan2(d[1], d[0]);
    // double vel = startVel;
    // double prevAngularVel = startAngularVel;

    // while (t < 1) {
    //     t += step / std::hypot(d[0], d[1]);
    //     d = path.d(t);
    //     double nextTheta = std::atan2(d[1], d[0]);
    //     double curvature = constrainAngle90(nextTheta - theta) / step;
    //     double angularVel = curvature * vel;
    //     theta = nextTheta;
    //     vels.push_back(vel);
    //     angularVels.push_back(angularVel);

    //     double angularAcc = (angularVel - prevAngularVel) * (vel / step);
    //     angularAccels.push_back(angularAcc);
    //     prevAngularVel = angularVel;
    //     double prevVel = vel;
    //     vel = std::min(constraints.constrainedSpeed(curvature),
    //                    std::sqrt(vel * vel + 2 * constraints.constrainedAcc(curvature) * step));
    //     accels.push_back((vel - prevVel) * (prevVel / step));
    // }

    double t = 0;
    Vec2 d = path.d(t);
    double theta = std::atan2(d[1], d[0]);
    vels.push_back(startVel);
    angularVels.push_back(startAngularVel);
    // accels.push_back(0);
    // angularAccels.push_back(0);
    double prevVel = startVel;

    while (t < 1) {
        t += step / std::hypot(d[0], d[1]);
        d = path.d(t);
        double nextTheta = std::atan2(d[1], d[0]);
        double curvature = constrainAngle90(nextTheta - theta) / step;

        // double maxVel = std::min(2 * constraints.maxVel / (constraints.trackWidth * curvature),
        //                          constraints.constrainedSpeed(curvature));
        // double maxAcc = std::min(2 * constraints.maxAcc / (constraints.trackWidth * curvature),
        //                          constraints.constrainedAcc(curvature));
        // double vel = std::min(maxVel, std::sqrt(prevVel * prevVel + 2 * maxAcc * step));

        // TODO: asdfdf prevVel
        double vel = std::min(constraints.constrainedSpeed(curvature),
                              std::sqrt(prevVel * prevVel + 2 * constraints.constrainedAcc(curvature) * step));
        double angularVel = curvature * vel;
        double acc = (vel - prevVel) * (prevVel / step);
        double angularAcc = curvature * acc;

        prevVel = vel;
        theta = nextTheta;

        vels.push_back(vel);
        angularVels.push_back(angularVel);
        accels.push_back(acc);
        angularAccels.push_back(angularAcc);
    }

    // t = 1;
    // d = path.d(t);
    // vel = endVel;
    // prevAngularVel = endAngularVel;

    // vels.push_back(vel);
    // angularVels.push_back(0);
    // accels.push_back(0);
    // angularAccels.push_back(0);

    // int i = vels.size() - 1;
    // while (t > 0) {
    //     t -= step / std::hypot(d[0], d[1]);
    //     d = path.d(t);
    //     double nextTheta = std::atan2(d[1], d[0]);
    //     double curvature = -constrainAngle90(nextTheta - theta) / step;
    //     double angularVel = curvature * vel;
    //     theta = nextTheta;
    //     if (std::abs(vel) < std::abs(vels[i])) { vels[i] = vel; }
    //     if (std::abs(angularVel) < std::abs(angularVels[i])) { angularVels[i] = angularVel; }

    //     double angularAcc = -(angularVel - prevAngularVel) * (vel / step);
    //     if (std::abs(angularAcc) > std::abs(angularAccels[i])) { angularAccels[i] = angularAcc; }
    //     prevAngularVel = angularVel;
    //     double maxAcc = constraints.maxDec - std::abs(angularAcc * constraints.trackWidth / 2);
    //     double prevVel = vel;
    //     vel = std::min(constraints.constrainedSpeed(curvature),
    //                    std::sqrt(vel * vel + 2 * constraints.constrainedAcc(curvature) * step));
    //     double accel = -(vel - prevVel) * (prevVel / step);
    //     if (std::abs(accel) > std::abs(accels[i])) { accels[i] = accel; }
    //     i--;
    // }

    t = 1;
    d = path.d(t);
    theta = std::atan2(d[1], d[0]);
    vels.push_back(endVel);
    angularVels.push_back(endAngularVel);
    accels.resize(vels.size());
    angularAccels.resize(vels.size());
    prevVel = endVel;

    int i = vels.size() - 1;
    while (t > 0) {
        t -= step / std::hypot(d[0], d[1]);
        d = path.d(t);
        double nextTheta = std::atan2(d[1], d[0]);
        double curvature = constrainAngle90(nextTheta - theta) / step;
        double vel = std::min(constraints.constrainedSpeed(curvature),
                              std::sqrt(prevVel * prevVel + 2 * constraints.constrainedDec(curvature) * step));
        double angularVel = curvature * vel;
        double acc = -(vel - prevVel) * (prevVel / step);
        double angularAcc = curvature * acc;

        prevVel = vel;
        theta = nextTheta;

        // TODO: is this even right
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
