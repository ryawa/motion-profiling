#include <chrono>
#include <ratio>
#include <iostream>
#include "Eigen/Dense"
#include "Constraints.hpp"
#include "Path.hpp"
#include "Trajectory.hpp"

int main() {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;

    QuinticHermite qh {
        {0, 0},
        {30, 30},
        {48, 0},
        {30, 30},
    };

    Constraints constraints {100, 500, 500, 12.1875, 1};
    Trajectory trajectory {qh, constraints, 0.1};

    auto t1 = high_resolution_clock::now();
    trajectory.generate(0, 0, 0, 0);
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> dt = t2 - t1;
    std::cout << "TIME: " << dt.count() << "\n\n";

    std::cout << trajectory.vels.size() << " " << trajectory.angularVels.size() << " " << trajectory.accels.size()
              << " " << trajectory.angularAccels.size() << "\n\n";

    std::cout << "VELS: ";
    for (int i = 0; i < trajectory.vels.size(); i++) { std::cout << trajectory.vels[i] << " "; }
    std::cout << "\n\n";

    std::cout << "ANGULARVELS: ";
    for (int i = 0; i < trajectory.angularVels.size(); i++) { std::cout << trajectory.angularVels[i] << " "; }
    std::cout << "\n\n";

    std::cout << "ACCELS: ";
    for (int i = 0; i < trajectory.accels.size(); i++) { std::cout << trajectory.accels[i] << " "; }
    std::cout << "\n\n";

    std::cout << "ANGULARACCELS: ";
    for (int i = 0; i < trajectory.angularAccels.size(); i++) { std::cout << trajectory.angularAccels[i] << " "; }
    std::cout << "\n";
}
