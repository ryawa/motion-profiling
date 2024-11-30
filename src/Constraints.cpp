#include <cmath>
#include "Constraints.hpp"

float Constraints::constrainedSpeed(float curvature) {
    float maxTurnSpeed = 2 * maxVel / (2 + std::abs(curvature) * trackWidth);
    if (curvature == 0) { return maxTurnSpeed; }
    float maxSlipSpeed = std::sqrt(frictionCoeff / std::abs(curvature) * 9.81);
    return std::min(maxTurnSpeed, maxSlipSpeed);
}

float Constraints::constrainedAcc(float curvature) { return 2 * maxAcc / (trackWidth * std::abs(curvature) + 2); }

float Constraints::constrainedDec(float curvature) { return 2 * maxDec / (trackWidth * std::abs(curvature) + 2); }
