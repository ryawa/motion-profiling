#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <vector>

class Profiler {
    public:
        Profiler(lemlib::Chassis* chassis, lemlib::Drivetrain* drivetrain, const float b, const float zeta,
                 const float scrubFactor);
        void follow(const asset& profile);
        void waitUntil(float d);
        void waitUntilDone();
    private:
        const float b;
        const float zeta;
        const float scrubFactor;

        lemlib::Chassis* chassis;
        lemlib::Drivetrain* drivetrain;

        float step;
        std::vector<float> vels;
        std::vector<float> curvatures;
        std::vector<lemlib::Pose> desiredPoses;

        bool isRunning;
        float distTraveled;

        pros::Mutex m;
        pros::Task t;
        void loop();
};
