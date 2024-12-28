#include "Pose.hpp"
#include <iostream>
#include <sstream>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

const uint8_t* buf = reinterpret_cast<const uint8_t*>(
    "3 0 0.10002 1.76003e-06 5.25924e-05 \n5.3799 0.000525924 0.200099 1.39264e-05 0.000207056 \n");

int main() {
    // std::stringstream s;
    // s << buf;
    std::vector<float> vels;
    std::vector<float> curvatures;
    std::vector<Pose> desiredPoses;
    // std::istringstream is;
    // for (std::string line; std::getline(s, line);) {
    //     is.clear();
    //     is.str(line);
    //     float vel;
    //     float curvature;
    //     Pose pose;
    //     is >> vel >> curvature >> pose.x >> pose.y >> pose.theta;
    //     vels.push_back(vel);
    //     curvatures.push_back(curvature);
    //     desiredPoses.push_back(pose);
    // }
    float step;
    std::stringstream s;
    s << buf;
    s >> step;
    std::istringstream is;
    std::cout << "HI\n";
    int i = 0;
    for (std::string line; std::getline(s, line);) {
        std::cout << i << "\n";
        is.clear();
        is.str(line);

        float vel;
        float curvature;
        float x;
        float y;
        float theta;
        is >> vel >> curvature >> x >> y >> theta;

        vels.push_back(vel);
        curvatures.push_back(curvature);
        Pose p {x, y, theta};
        desiredPoses.push_back(p);
        i++;
    }
    std::cout << "HI2";

    for (int i = 0; i < vels.size(); i++) {
        std::cout << vels[i] << " " << curvatures[i] << " " << desiredPoses[i].x << " " << desiredPoses[i].y << " "
                  << desiredPoses[i].theta << "\n";
    }
}
