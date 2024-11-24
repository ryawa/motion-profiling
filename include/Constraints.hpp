#pragma once

struct Constraints {
        double maxVel;
        double maxAcc;
        double maxDec;
        double trackWidth;
        double frictionCoeff;
        double constrainedSpeed(double curvature);
        double constrainedAcc(double curvature);
        double constrainedDec(double curvature);
};
