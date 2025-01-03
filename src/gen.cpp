#include <chrono>
#include <ratio>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "Constraints.hpp"
#include "Path.hpp"
#include "Trajectory.hpp"

int main() {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;

    Path* path = new QuinticHermiteSpline {
        {
            {
                {0, 0},
                {30, 0},
                {72, 72},
                {0, 180},
            },
        },
    };

    Constraints constraints {75, 100, 50, 10.8125, 0.02};
    Trajectory trajectory {path, constraints, 0.1};

    auto t1 = high_resolution_clock::now();
    trajectory.generate(3, 0, 0, 0);
    auto t2 = high_resolution_clock::now();
    duration<float, std::milli> dtime = t2 - t1;
    // std::cout << "TIME: " << dtime.count() << "\n\n";

    std::cout << trajectory.step << "\n";

    for (int i = 0; i < trajectory.vels.size(); i++) {
        std::cout << trajectory.vels[i] << " ";
        std::cout << trajectory.curvatures[i] << " ";
        std::cout << trajectory.desiredPoses[i].x << " ";
        std::cout << trajectory.desiredPoses[i].y << " ";
        std::cout << trajectory.desiredPoses[i].theta << " ";
        std::cout << "\n";
    }
}
