#pragma once

#include "Constraints.hpp"
#include "Path.hpp"

class Trajectory {
    public:
        // TODO: abstract Path class
        Trajectory(QuinticHermite path, Constraints constraints, double step);
        void generate(double startVel, double endVel, double startAngularVel, double endAngularVel);
        std::vector<double> vels;
        std::vector<double> accels;
        std::vector<double> angularVels;
        std::vector<double> angularAccels;
        double step;
    private:
        QuinticHermite path;
        Constraints constraints;

        double constrainAngle180(double angle);
        double constrainAngle90(double angle);
};
