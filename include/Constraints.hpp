#pragma once

struct Constraints {
        double maxVel;
        double maxAcc;
        double maxDec;
        double trackWidth;
        double frictionCoeff;
        double maxSpeed(double curvature);
};
