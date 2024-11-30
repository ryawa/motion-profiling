#pragma once

struct Constraints {
        float maxVel;
        float maxAcc;
        float maxDec;
        float trackWidth;
        float frictionCoeff;
        float constrainedSpeed(float curvature);
        float constrainedAcc(float curvature);
        float constrainedDec(float curvature);
};
