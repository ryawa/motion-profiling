#include <cmath>
#include "Constraints.hpp"

double Constraints::maxSpeed(double curvature) {
    double maxTurnSpeed = 2 * maxVel / (2 + std::abs(curvature) * trackWidth);
    if (curvature == 0) { return maxTurnSpeed; }
    double maxSlipSpeed = std::sqrt(frictionCoeff / std::abs(curvature) * 9.81);
    return std::min(maxTurnSpeed, maxSlipSpeed);
}
