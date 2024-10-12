#pragma once

#include "Constraints.hpp"
#include "Path.hpp"

class Trajectory {
    public:
        // TODO: abstract Path class
        Trajectory(QuinticHermite path, Constraints constraints, double step);
        void generate(double startVel, double endVel, double startAngularVel, double endAngularVel);
        std::vector<double> vels;
        std::vector<double> angularVels;
        double step;
    private:
        QuinticHermite path;
        Constraints constraints;
};
