#include <cmath>
#include "Constraints.hpp"

// TODO: abs is correct?
double Constraints::constrainedSpeed(double curvature) {
    double maxTurnSpeed = 2 * maxVel / (2 + std::abs(curvature) * trackWidth);
    if (curvature == 0) { return maxTurnSpeed; }
    double maxSlipSpeed = std::sqrt(frictionCoeff / std::abs(curvature) * 9.81);
    return std::min(maxTurnSpeed, maxSlipSpeed);
}

double Constraints::constrainedAcc(double curvature) { return 2 * maxAcc / (trackWidth * std::abs(curvature) + 2); }

double Constraints::constrainedDec(double curvature) { return 2 * maxDec / (trackWidth * std::abs(curvature) + 2); }
