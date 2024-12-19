#pragma once

#include <utility>
#include <vector>

#include "Constraints.hpp"
#include "Path.hpp"
#include "Pose.hpp"

class Trajectory {
    public:
        Trajectory(Path* path, Constraints constraints, float step);
        void generate(float startVel, float endVel, float startAngularVel, float endAngularVel);
        std::pair<float, float> getWheelVelocities(int i, float wheelDiameter);
        std::vector<float> vels;
        std::vector<float> curvatures;
        std::vector<Pose> desiredPoses;
        float step;
    private:
        Path* path;
        Constraints constraints;

        float constrainAngle180(float angle);
        float constrainAngle90(float angle);
};
